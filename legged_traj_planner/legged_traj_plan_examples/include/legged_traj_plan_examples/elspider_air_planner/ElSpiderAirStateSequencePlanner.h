/**
 * @file ElSpiderAirStateSequencePlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
// MCTS
#include "contactPlannerInterface.h"
#include "planning.h"
#include "myDataType.h"
#include "HexapodParameter.h"
#include "user.h"

#include <pinocchio/math/rpy.hpp>
#include "legged_traj_plan/whole_body_planner/CmdVelExtrapolator.h"
#include "legged_traj_plan/whole_body_planner/StateSequencePlanner.h"
#include "ElSpiderAirPlannerBase.h"

#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/BodyState.h"

/* external project header files */
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "legged_traj_search/utils/gcs_visualizer.hpp"

legged_traj_plan::hexapod_State transRobotState(const MDT::RobotState &state_)
{
    legged_traj_plan::hexapod_State hexapodState;
    hexapodState.base_Pose_Now.position.x = state_.pose.x;
    hexapodState.base_Pose_Now.position.y = state_.pose.y;
    hexapodState.base_Pose_Now.position.z = state_.pose.z;
    hexapodState.base_Pose_Now.orientation.roll = state_.pose.roll;
    hexapodState.base_Pose_Now.orientation.pitch = state_.pose.pitch;
    hexapodState.base_Pose_Now.orientation.yaw = state_.pose.yaw;
    hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;

    for (int i = 0; i < 6; i++)
    {
        hexapodState.feetPositionNow.foot[i].x = state_.feetPosition[i].x();
        hexapodState.feetPositionNow.foot[i].y = state_.feetPosition[i].y();
        hexapodState.feetPositionNow.foot[i].z = state_.feetPosition[i].z();
        hexapodState.support_State_Now[i] = !state_.gaitToNow[i];
        hexapodState.faultLeg_State_Now[i] = MDT::NORMAL_LEG_FLAG;
    }
    hexapodState.move_Direction.x = cos(state_.moveDirection);
    hexapodState.move_Direction.y = sin(state_.moveDirection);
    hexapodState.move_Direction.z = 0;
    // 下一步落足点
    hexapodState.feetPositionNext = hexapodState.feetPositionNow;
    // 下一步支撑状态和容错状态
    hexapodState.support_State_Next = hexapodState.support_State_Now;
    hexapodState.faultLeg_State_Next = hexapodState.faultLeg_State_Now;
    return hexapodState;
}

// 初始化机器人状态,并赋初值
MDT::RobotState initRobotState(Parameters &param, const MDT::Pose &robotPoseW, MDT::Vector6b gaitToNow, float moveDirection)
{
    MDT::RobotState state_;
    state_.initialize();
    state_.pose = robotPoseW;
    state_.faultStateToNow << MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG;
    for (int i = 0; i < 6; i++)
    {
        // PLANNING::POINT pnt = {HexapodParameter::transList[i].x, HexapodParameter::transList[i].y, HexapodParameter::transList[i].z};
        state_.feetPosition[i] = MDT::pointRotationAndTrans(param.norminalFoothold_B[i], robotPoseW.getT_W_B());
        state_.feetNormalVector[i] << 0, 0, 1; // 默认法向量竖直向上
        state_.gaitToNow[i] = gaitToNow[i];
        state_.maxNormalForce[i] = 1000.0f;
        state_.frcitionMu[i] = 0.8f;
    }
    state_.moveDirection = moveDirection;

    return state_;
}

MDT::RobotState getInitState(Parameters &param, MDT::Pose robotPose = {0, 0, USER::norminalTrunkHeight, 0, 0, -1.5 * _PI_ / 6},
                             float moveDir = 0)
{
    MDT::Vector6b gaitToNow;
    gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
    return initRobotState(param, robotPose, gaitToNow, moveDir);
}

// For robot state recording
struct RobotProfile
{
    double time;
    double t; // param time in traj
    pinocchio::SE3 pose;
    PosList foot_pos_list;
    PosList cfg_pos_list;
    PosList cfg_vel_list;
    std::array<double, 6> foot_end_sdf;
    std::array<bool, 6> support_state;
    geometry_msgs::Twist cmd_vel;
};

namespace DemoFiles
{
    const std::string planned_states = "planned_states";
};

struct ElSpiderAirStateSequencePlannerConfig
{
    int rosRate;
    double stepTime;

    int cmdMctsSearchNodeNum;
    int navMctsSearchNodeNum;

    int cmdExtrapolatePointNum;
    float cmdExtrapolateDeltaT;
    int navExtrapolateSamplesNum;

    bool enableReachableFiltering;
    double reachableFilterPoseMoveDis;
    std::string reachableTravLayerName;

    bool visReachableCheck;

    bool swingTrajPreOpt;
    bool shutdownAfterPreOpt;
    bool execOnKeyboardCmd;

    std::string demoPath;
    bool savePlannedStates;
    bool execSavedStates;

    std::string OptBenchmarkSavePath;

    void loadParams(ros::NodeHandle &nh, std::string ns = "StateSequencePlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/rosRate", rosRate);
        check_digit &= nh.getParam(ns + "/stepTime", stepTime);
        check_digit &= nh.getParam(ns + "/cmdMctsSearchNodeNum", cmdMctsSearchNodeNum);
        check_digit &= nh.getParam(ns + "/navMctsSearchNodeNum", navMctsSearchNodeNum);
        check_digit &= nh.getParam(ns + "/cmdExtrapolatePointNum", cmdExtrapolatePointNum);
        check_digit &= nh.getParam(ns + "/cmdExtrapolateDeltaT", cmdExtrapolateDeltaT);
        check_digit &= nh.getParam(ns + "/navExtrapolateSamplesNum", navExtrapolateSamplesNum);

        check_digit &= nh.getParam(ns + "/enableReachableFiltering", enableReachableFiltering);
        check_digit &= nh.getParam(ns + "/reachableFilterPoseMoveDis", reachableFilterPoseMoveDis);
        check_digit &= nh.getParam(ns + "/reachableTravLayerName", reachableTravLayerName);

        check_digit &= nh.getParam(ns + "/visReachableCheck", visReachableCheck);

        check_digit &= nh.getParam(ns + "/swingTrajPreOpt", swingTrajPreOpt);
        check_digit &= nh.getParam(ns + "/shutdownAfterPreOpt", shutdownAfterPreOpt);
        check_digit &= nh.getParam(ns + "/execOnKeyboardCmd", execOnKeyboardCmd);

        check_digit &= nh.getParam(ns + "/demoPath", demoPath);
        check_digit &= nh.getParam(ns + "/savePlannedStates", savePlannedStates);
        check_digit &= nh.getParam(ns + "/execSavedStates", execSavedStates);

        check_digit &= nh.getParam(ns + "/OptBenchmarkSavePath", OptBenchmarkSavePath);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load ElSpiderAirStateSequencePlannerConfig.");
        }
    }
};

class ElSpiderAirStateSequencePlanner : public ElSpiderAirPlannerBase
{
private:
    ros::Rate rate_;

    ElSpiderAirStateSequencePlannerConfig config_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;
    ros::Subscriber nav_sub_;
    geometry_msgs::PoseStamped nav_;

    // Interface
    StateSequencePlanner state_sequence_planner_;

    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<MDT::RobotState> planned_states_;

    GridMapCmdVelExtrapolator gridmap_extrapolator_;
    std::vector<Eigen::Vector3f> exp_path_;

    // Status
    bool motion_lock_ = false;

    // Benchmarking
    double init_time_ = 0.0;
    std::vector<RobotProfile> robot_profile_;
    Benchmark benchmark_;
    ros::Publisher benchmark_progress_pub_;

    // Visualizer
    GCSVisualizer visualizer_;
    GCSVisualizer visualizer_base_;
    ros::Publisher gridmap_pub_;

public:
    ElSpiderAirStateSequencePlanner() : ElSpiderAirPlannerBase(),
                                        state_sequence_planner_(swing_traj_planner_, gridmap_interface_, robot_interface_),
                                        visualizer_(nh_, "odom", "visualizer_markers"),
                                        visualizer_base_(nh_, "base", "visualizer_markers_base"),
                                        rate_(100), benchmark_("ElSpiderAirStateSequencePlannerBenchmark",
                                                               swing_traj_planner_config_.enableBenchmark)
    {
        config_.loadParams(nh_);
        rate_ = ros::Rate(config_.rosRate);
        init_time_ = ros::Time::now().toSec();
        benchmark_progress_pub_ = nh_.advertise<std_msgs::Bool>("/benchmark_progress", 1);
        gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_trav_test2", 1, true);
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirStateSequencePlanner::cmd_callback, this);
        nav_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ElSpiderAirStateSequencePlanner::nav_callback, this);
        state_sequence_planner_.enableRecordStates(config_.savePlannedStates);

        PosList pose_sample_pts;
        int len = 6;
        double delta = 0.2;
        for (int i = 0; i < len; i++)
        {
            for (int j = 0; j < len; j++)
            {
                pose_sample_pts.emplace_back(Eigen::Vector3d(i * delta - 0.5 * len * delta,
                                                             j * delta - 0.5 * len * delta, 0));
            }
        }
        gridmap_extrapolator_.init(gridmap_interface_, pose_sample_pts);

        robot_state_.initialize();
        next_planned_state_.initialize();
        update_robot_state(robot_interface_->getBodyPoseFdb(), robot_interface_->getFootStateFdb());
        next_planned_state_ = robot_state_;

        if (config_.execSavedStates)
        {
            ros::Duration(1.0).sleep();
            execRecordStates();
        }
    }

    grid_map::GridMap &gridmapReachableFiltering(const geometry_msgs::Twist cmd_vel, double move_dis = 0.0)
    {
        grid_map::GridMap &map = gridmap_interface_->getMap();
        pinocchio::SE3 pose0 = robot_interface_->getBodyPoseFdb();
        pinocchio::SE3 pose1 = pose0;
        Eigen::Vector3d vel = Eigen::Vector3d(cmd_vel.linear.x, cmd_vel.linear.y, 0);
        vel.normalize();
        pose1.translation() += vel * move_dis;

        std::shared_ptr<FLTCfgPlanner> flt_planner = std::dynamic_pointer_cast<FLTCfgPlanner>(swing_traj_planner_);
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        std::vector<Eigen::Vector3d> foot_pos;
        std::vector<Eigen::Vector3d> gridmap_points;
        std::vector<bool> gridmap_points_valid[6];
        for (int i = 0; i < 6; i++)
            foot_pos.emplace_back(point_SE3Act(pose0.inverse(), Eigen::Vector3d(foot_state.position[i].x, foot_state.position[i].y, foot_state.position[i].z)));

        try
        {
            if (config_.reachableTravLayerName != gridmap_interface_->getTravLayerName())
                map.add(config_.reachableTravLayerName, map.get(gridmap_interface_->getTravLayerName()));
            for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
            {
                Eigen::Vector3d p;
                map.getPosition3(config_.reachableTravLayerName, *iterator, p);
                gridmap_points.emplace_back(p);
            }

            for (int i = 0; i < 6; i++)
            {
                flt_planner->reachableCheckHook(pose0, pose1, foot_pos[i], i,
                                                gridmap_points, gridmap_points_valid[i]);
            }

            int i = 0;
            for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
            {
                bool valid = false;
                for (int index = 0; index < 6; index++)
                {
                    valid |= gridmap_points_valid[index][i];
                }
                if (!valid)
                    map.at(config_.reachableTravLayerName, *iterator) = std::nan("");
                i++;
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("Failed to update trav map!");
        }
        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map, msg);
        gridmap_pub_.publish(msg);
        return map;
    }

    // cmd_vel callback
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        if (motion_lock_)
            ROS_WARN("Robot is in motion, ignore new command.");
        else
        {
            motion_lock_ = true;
            gridmap_interface_->lockMapUpdate();
            cmd_ = msg;
            state_sequence_planner_.visClear();
            Parameters param(swing_traj_planner_config_.plannerID == 0 && config_.enableReachableFiltering ? gridmapReachableFiltering(cmd_, config_.reachableFilterPoseMoveDis) : gridmap_interface_->getMap());
            bool ret = false;

            while (!ret && ros::ok())
            {
                // Fetch feedback
                ros::spinOnce();
                // update robot state
                update_robot_state(robot_interface_->getBodyPoseFdb(), robot_interface_->getFootStateFdb());
                // update_exp_path(cmd_);
                update_exp_path_xlock();
                // MCTS planning
                ret = CONTACT_PLANNER::pathTrackPlanner(robot_state_, next_planned_state_, exp_path_,
                                                        param, true, config_.cmdMctsSearchNodeNum);
                // next_planned_state_ = CONTACT_PLANNER::tripleGaitPlanner(robot_state_, gridmap_interface_->getMap(), 0.1);
            }

            // Visualization
            visualizer_.delAll();
            visualizer_base_.delAll();

            if (swing_traj_planner_config_.enableVis)
            {
                // Vis expected path
                std::vector<Point3D> exp_path_vis;
                for (const auto &pt : exp_path_)
                {
                    exp_path_vis.emplace_back(Point3D(pt[0], pt[1], pt[2]));
                }
                visualizer_.visCurve(exp_path_vis);

                // // Vis getAvailableFootholds
                // MDT::AvailableContactsInfo available_points = PLANNING::getAvailableFootholds(next_planned_state_, gridmap_interface_->getMap());
                // std::vector<Eigen::Vector3d> pts;
                // for (int i = 0; i < 6; i++)
                // {
                //     if (next_planned_state_.gaitToNow[i] == MDT::SUPPORT_FLAG)
                //         continue;
                //     for (auto pt : available_points.position.leg[i])
                //     {
                //         pts.emplace_back(pt);
                //     }
                // }
                // visualizer_.visSphere(pts, 0.01);
            }

            if (ret)
            {
                state_sequence_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                            transRobotState(next_planned_state_));
            }
            else
            {
                ROS_INFO("MCTS failed to plan, reset to nominal state.");
                visualizer_base_.visPolytope(robot_interface_->getFootPolyhedra());
                state_sequence_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                            transRobotState(getInitState(param, robot_state_.pose, robot_state_.moveDirection)));
            }
            // state_traj_replay(state_sequence_planner_.get_state_traj(0));
            traj_planner();
            motion_lock_ = false;
            gridmap_interface_->unlockMapUpdate();
        }
    }

    void nav_callback(const geometry_msgs::PoseStamped &msg)
    {
        if (motion_lock_)
            ROS_WARN("Robot is in motion, ignore new command.");
        else
        {
            motion_lock_ = true;
            gridmap_interface_->lockMapUpdate();

            nav_ = msg;
            state_sequence_planner_.visClear();

            Parameters param(gridmap_interface_->getMap());

            bool ret = false;
            while (!ret && ros::ok())
            {
                // Fetch feedback
                ros::spinOnce();
                // update robot state
                update_robot_state(robot_interface_->getBodyPoseFdb(), robot_interface_->getFootStateFdb());
                update_exp_path(nav_);
                planned_states_.clear();

                // MCTS planning
                ret = CONTACT_PLANNER::pathTrackPlanner(robot_state_, planned_states_, exp_path_,
                                                        param, config_.navMctsSearchNodeNum);
                // next_planned_state_ = CONTACT_PLANNER::tripleGaitPlanner(robot_state_, gridmap_interface_->getMap(), 0.1);
            }

            // Visualization
            visualizer_.delAll();
            visualizer_base_.delAll();

            if (swing_traj_planner_config_.enableVis)
            {
                // Vis expected path
                std::vector<Point3D> exp_path_vis;
                for (const auto &pt : exp_path_)
                {
                    exp_path_vis.emplace_back(Point3D(pt[0], pt[1], pt[2]));
                }
                visualizer_.visCurve(exp_path_vis);
            }

            if (ret)
            {
                state_sequence_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                            transRobotState(planned_states_.at(0)));
                for (size_t i = 1; i < planned_states_.size(); ++i)
                {
                    state_sequence_planner_.enqueue_MCTsolution(transRobotState(planned_states_.at(i - 1)),
                                                                transRobotState(planned_states_.at(i)));
                }
            }
            else
            {
                ROS_INFO("MCTS failed to plan, reset to nominal state.");
                visualizer_base_.visPolytope(robot_interface_->getFootPolyhedra());
                state_sequence_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                            transRobotState(getInitState(param, robot_state_.pose, robot_state_.moveDirection)));
            }
            states_replay(state_sequence_planner_);
            traj_planner();
            motion_lock_ = false;
            gridmap_interface_->unlockMapUpdate();
        }
    }

    //// Rviz
    void state_traj_replay(MCTStateTransfer &state_traj)
    {
        for (double t = 0.0; t < 1.01; t += 0.01)
        {
            // Get Interpolated State
            auto odom_interp = state_traj.eval_torso_traj(sine_remap(t));
            auto footend_interp = state_traj.eval_cfg_traj(sine_remap(t));
            auto support_state = state_traj.eval_support_state(t);

            // Visualization
            robot_interface_shadow_->setBodyPoseCmd(odom_interp);
            robot_interface_shadow_->setJointCmd(footend_interp);

            ros::spinOnce(); // Fetch feedback
            rate_.sleep();
        }
    }

    void states_replay(StateSequencePlanner &wbplanner)
    {
        for (size_t i = 0; i < wbplanner.get_state_traj_length(); ++i)
        {
            MCTStateTransfer &state_traj = wbplanner.get_state_traj(i);
            auto odom_interp = state_traj.eval_torso_traj(0);
            auto footend_interp = state_traj.eval_cfg_traj(0);
            auto support_state = state_traj.eval_support_state(0);

            // Visualization
            robot_interface_shadow_->setBodyPoseCmd(odom_interp);
            robot_interface_shadow_->setJointCmd(footend_interp);

            ros::spinOnce(); // Fetch feedback
            ros::Duration(0.2).sleep();
        }
    }

    //// MCTS Interface
    // Update Robot State for MCTS Interface
    void update_robot_state(pinocchio::SE3 body_pose, legged_traj_plan::FootState foot_state)
    {
        // Robot pose
        robot_state_.pose.x = body_pose.translation()[0];
        robot_state_.pose.y = body_pose.translation()[1];
        robot_state_.pose.z = body_pose.translation()[2];
        Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(body_pose.rotation());
        robot_state_.pose.roll = rpy[0];
        robot_state_.pose.pitch = rpy[1];
        robot_state_.pose.yaw = rpy[2];

        // Foot state
        for (int i = 0; i < 6; ++i)
        {
            // Last planned state gait (MDT::SUPPORT if not planned)
            robot_state_.gaitToNow[i] = next_planned_state_.gaitToNow[i];
            robot_state_.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
            // Absolute foot position
            robot_state_.feetPosition[i] = point_SE3Act(body_pose.inverse(), Eigen::Vector3d(foot_state.position[i].x, foot_state.position[i].y, foot_state.position[i].z));
            robot_state_.feetNormalVector[i] << 0, 0, 1; // TODO: use gridmap normal
        }
        robot_state_.moveDirection = robot_state_.pose.yaw;
    }

    // Update Exp Path for MCTS Interface
    void update_exp_path(const geometry_msgs::Twist &cmd)
    {
        exp_path_.clear();
        gridmap_extrapolator_.update(robot_interface_->getBodyPoseFdb(), cmd);

        for (int i = 0; i < config_.cmdExtrapolatePointNum; ++i)
        {
            Eigen::Vector3d trans = gridmap_extrapolator_.extrapolate(i * config_.cmdExtrapolateDeltaT).translation();
            exp_path_.push_back(Eigen::Vector3f(trans[0], trans[1], trans[2]));
        }
    }

    void update_exp_path_xlock(void)
    {
        exp_path_.clear();
        double height = 0.25;
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x, robot_state_.pose.y, height));
        for (int i = 0; i < config_.cmdExtrapolatePointNum; ++i)
        {
            exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + cmd_.linear.x * config_.cmdExtrapolateDeltaT * i,
                                                robot_state_.pose.y,
                                                height));
        }
    }

    void update_exp_path(const geometry_msgs::PoseStamped &nav)
    {
        exp_path_.clear();
        pinocchio::SE3 goal_pose(Eigen::Quaterniond(nav.pose.orientation.w, nav.pose.orientation.x, nav.pose.orientation.y, nav.pose.orientation.z),
                                 Eigen::Vector3d(nav.pose.position.x, nav.pose.position.y, nav.pose.position.z));
        gridmap_extrapolator_.update(goal_pose);
        // adapt to the gridmap
        goal_pose = gridmap_extrapolator_.extrapolate(0);
        pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
        pinocchio::SE3 error_pose = body_pose.inverse() * goal_pose;
        pinocchio::Motion error_motion = pinocchio::log6(error_pose);
        for (int i = 0; i < config_.navExtrapolateSamplesNum; ++i)
        {
            double t = static_cast<double>(i) / config_.navExtrapolateSamplesNum;
            pinocchio::SE3 interp_pose = body_pose * pinocchio::exp6(t * error_motion);
            gridmap_extrapolator_.update(interp_pose);
            interp_pose = gridmap_extrapolator_.extrapolate(0);
            exp_path_.push_back(Eigen::Vector3f(interp_pose.translation()[0], interp_pose.translation()[1], interp_pose.translation()[2]));
        }
    }

    //// Contact Handling (Sim only)
    // FIXME: avoid error detection
    bool is_contact(int leg_idx, double eps = 0.1)
    {
        Eigen::Vector3d foot_force;
        auto foot_state_ = robot_interface_->getFootStateFdb();
        foot_force << foot_state_.effort[leg_idx].x, foot_state_.effort[leg_idx].y, foot_state_.effort[leg_idx].z;
        if (robot_interface_type_ == "ElSpiderAirDummy")
            return (foot_force.norm() > eps) || foot_state_.contact[leg_idx];
        else
            return (foot_force.norm() > eps);
    }

    void stance_contact_handle(void)
    {
        bool flag = false;
        int max_cnt = 200;
        double alpha = 0.005;
        ros::spinOnce(); // Fetch feedback
        auto foot_state_ = robot_interface_->getFootStateFdb();
        std::vector<Eigen::Vector3d> footend_interp(6, Eigen::Vector3d::Zero());
        for (size_t k = 0; k < 6; ++k)
        {
            footend_interp.at(k)[0] = foot_state_.position[k].x;
            footend_interp.at(k)[1] = foot_state_.position[k].y;
            footend_interp.at(k)[2] = foot_state_.position[k].z;
        }

        ROS_INFO("Stance contact handling...");
        while (!flag && max_cnt-- > 0)
        {
            flag = true;
            for (size_t k = 0; k < 6; ++k)
            {
                if (is_contact(k) == false)
                {
                    flag = false;
                    footend_interp.at(k) = LOWEST_FOOT_POS[k] * alpha + footend_interp.at(k) * (1 - alpha);
                }
            }
            if (swing_traj_planner_config_.useCfgCommand)
                robot_interface_->setJointCmd(robot_interface_->IKFast_foots(footend_interp));
            else
                robot_interface_->setFootCmd(footend_interp);

            ros::spinOnce(); // Fetch feedback
            rate_.sleep();
        }
        if (max_cnt <= 0)
            ROS_WARN("Stance contact handling failed.");
        else
            ROS_INFO("Stance contact handling done.");
    }

    //// Planning
    // for lift & touch smoothing
    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }

    void traj_planner()
    {

        if (config_.swingTrajPreOpt)
        {
            benchmark_.reset();
            // Swing Traj Optimization
            state_sequence_planner_.optSwingTraj();
            benchmark_.record("SwingTrajOptimization");
            // Reachability Check
            state_sequence_planner_.reachableCheck();
            benchmark_.record("ReachabilityCheck");
            benchmark_.end();
            if (config_.shutdownAfterPreOpt)
            {
                std_msgs::Bool msg;
                msg.data = true;
                benchmark_progress_pub_.publish(msg);
                ros::shutdown();
                return;
            }
        }

        double t = 0.0;
        double delta = 1 / config_.stepTime / config_.rosRate;

        // Auto opt next traj before exec
        MCTStateTransfer &state_traj = state_sequence_planner_.get_state_traj(0);
        pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(0.0);
        // std::vector<Eigen::Vector3d> footend_interp_vel = state_traj.eval_foot_traj(0.0, 1);
        // std::vector<Eigen::Vector3d> footend_interp_acc = state_traj.eval_foot_traj(0.0, 2);
        std::vector<Eigen::Vector3d> footend_interp_vel = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());
        std::vector<Eigen::Vector3d> footend_interp_acc = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());
        std::array<bool, 6> support_state = state_traj.eval_support_state(0.0);

        if (config_.visReachableCheck)
        {
            state_traj.reachable_check();
        }

        do
        {
            // Get Interpolated State
            odom_interp = state_traj.eval_torso_traj(sine_remap(t));
            support_state = state_traj.eval_support_state(sine_remap(t));
            std::vector<bool> contact = std::vector<bool>(support_state.begin(), support_state.end());

            if (swing_traj_planner_config_.useCfgCommand)
            {
                footend_interp = state_traj.eval_cfg_traj(sine_remap(t));

                robot_interface_->setJointCmd(footend_interp, contact);
                robot_interface_->setBodyPoseCmd(odom_interp);
            }
            else
            {
                // Footend position in world frame
                footend_interp = state_traj.eval_foot_traj(sine_remap(t));
                // footend_interp_vel = state_traj.eval_foot_traj(sine_remap(t), 1);
                // footend_interp_acc = state_traj.eval_foot_traj(sine_remap(t), 2);
                footend_interp_vel = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());
                footend_interp_acc = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());

                for (size_t k = 0; k < 6; ++k)
                {
                    // Convert to BASE
                    footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
                }

                robot_interface_->setFootCmd(footend_interp, footend_interp_vel,
                                             footend_interp_acc, contact);
                robot_interface_->setBodyPoseCmd(odom_interp);
            }

            // Wait Key
            if (t == 0.0)
                waitKey("Press any key with Enter to execute.");

            // State recording
            RobotProfile profile;
            profile.time = ros::Time::now().toSec() - init_time_;
            profile.t = t;
            profile.pose = odom_interp;
            profile.foot_pos_list = state_traj.eval_foot_traj(sine_remap(t));
            profile.cfg_pos_list = state_traj.eval_cfg_traj(sine_remap(t), 0, false);
            profile.cfg_vel_list = state_traj.eval_cfg_traj(sine_remap(t), 1, false);
            profile.support_state = support_state;
            for (size_t k = 0; k < 6; ++k)
            {
                profile.foot_end_sdf[k] = gridmap_interface_->sdfValue(profile.foot_pos_list[k]);
            }
            robot_profile_.emplace_back(profile);

            // Update param t
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                state_sequence_planner_.dequeue_MCTsolution();
                if (state_sequence_planner_.get_state_traj_length() > 0)
                {
                    state_traj = state_sequence_planner_.get_state_traj(0);
                    state_sequence_planner_.visClear();
                }
            }
            ros::spinOnce(); // Fetch feedback
            rate_.sleep();
        } while (state_sequence_planner_.get_state_traj_length() > 0 && ros::ok());

        // Set all foot contact to true
        if (swing_traj_planner_config_.useCfgCommand)
            robot_interface_->setJointCmd(footend_interp, std::vector<bool>(6, true));
        else
            robot_interface_->setFootCmd(footend_interp, footend_interp_vel, footend_interp_acc,
                                         std::vector<bool>(6, true));

        // Stance contact handling
        stance_contact_handle();
    }

    //// Benchmarking
    void saveRobotProfile()
    {
        // Save to file
        std::ofstream file(swing_traj_planner_config_.robotProfilePath);
        if (file.is_open())
        {
            file << "time,t,pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,";
            file << "foot0_x,foot0_y,foot0_z,foot1_x,foot1_y,foot1_z,foot2_x,foot2_y,foot2_z,";
            file << "foot3_x,foot3_y,foot3_z,foot4_x,foot4_y,foot4_z,foot5_x,foot5_y,foot5_z,";
            file << "cfg0_x,cfg0_y,cfg0_z,cfg1_x,cfg1_y,cfg1_z,cfg2_x,cfg2_y,cfg2_z,";
            file << "cfg3_x,cfg3_y,cfg3_z,cfg4_x,cfg4_y,cfg4_z,cfg5_x,cfg5_y,cfg5_z,";
            file << "cfg0_dx,cfg0_dy,cfg0_dz,cfg1_dx,cfg1_dy,cfg1_dz,cfg2_dx,cfg2_dy,cfg2_dz,";
            file << "cfg3_dx,cfg3_dy,cfg3_dz,cfg4_dx,cfg4_dy,cfg4_dz,cfg5_dx,cfg5_dy,cfg5_dz,";
            file << "foot0_sdf,foot1_sdf,foot2_sdf,foot3_sdf,foot4_sdf,foot5_sdf,";
            file << "support0,support1,support2,support3,support4,support5\n";
            for (const auto &profile : robot_profile_)
            {
                file << profile.time << "," << profile.t << ",";
                auto pos = profile.pose.translation();
                file << pos[0] << "," << pos[1] << "," << pos[2] << ",";
                auto rpy = profile.pose.rotation().eulerAngles(0, 1, 2);
                file << rpy[0] << "," << rpy[1] << "," << rpy[2] << ",";
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.foot_pos_list[k][0] << "," << profile.foot_pos_list[k][1] << "," << profile.foot_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.cfg_pos_list[k][0] << "," << profile.cfg_pos_list[k][1] << "," << profile.cfg_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.cfg_vel_list[k][0] << "," << profile.cfg_vel_list[k][1] << "," << profile.cfg_vel_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.foot_end_sdf[k] << ",";
                }
                for (size_t k = 0; k < 5; ++k)
                {
                    file << profile.support_state[k] << ",";
                }
                file << profile.support_state[5] << "\n";
            }
        }
        file.close();
    }

    void saveRecordStates()
    {
        std::ofstream file(config_.demoPath + DemoFiles::planned_states, std::ios::binary);
        if (file.is_open())
        {
            std::vector<hexapod_State> record_states = state_sequence_planner_.getRecordStates();
            hexapod_State save_states[record_states.size()];
            for (size_t i = 0; i < record_states.size(); ++i)
            {
                save_states[i] = record_states[i];
            }
            file.write((char *)save_states, sizeof(hexapod_State) * record_states.size());
        }
        file.close();
    }

    void execRecordStates()
    {
        std::ifstream file(config_.demoPath + DemoFiles::planned_states, std::ios::binary);
        if (file.is_open())
        {
            std::vector<hexapod_State> record_states;
            hexapod_State *state = new hexapod_State();
            while (!file.eof())
            {
                file.read((char *)state, sizeof(hexapod_State));
                record_states.push_back(*state);
            }
            file.close();
            record_states.pop_back(); // FIXME: the last one is invalid
            ROS_INFO("Loaded record states: %ld", record_states.size());
            if (record_states.size() > 1)
            {
                for (size_t i = 0; i < record_states.size() - 1; ++i)
                {
                    state_sequence_planner_.enqueue_MCTsolution(record_states[i], record_states[i + 1]);
                }
                traj_planner();
                return;
            }
            else
            {
                ROS_WARN("No record states loaded.");
            }
        }
        else
        {
            ROS_WARN("Failed to open record states file.");
        }
    }

    char waitKey(std::string info = "Press any key with Enter to continue.")
    {
        if (config_.execOnKeyboardCmd)
        {
            std::cout << info << std::endl;
            return getchar();
        }
        return 0;
    }

    void run()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
        if (swing_traj_planner_config_.enableBenchmark)
        {
            std::cout << "Save benchmark results..." << std::endl;
            state_sequence_planner_.saveBenchmarkResults();
            benchmark_.save(config_.OptBenchmarkSavePath);
            std::cout << "Save robot profile..." << std::endl;
            saveRobotProfile();
            std::cout << "Benchmark results and robot profile saved." << std::endl;
        }
        if (config_.savePlannedStates)
        {
            saveRecordStates();
            std::cout << "Save planned states." << std::endl;
        }
    }
};