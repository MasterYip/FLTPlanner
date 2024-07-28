/**
 * @file ElSpiderAirSimplePlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h" // Should be included first (pinocchio)
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/whole_body_planner/WholeBodyPlanner.h"

#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/BodyState.h"
// MCTS
#include "contactPlannerInterface.h"
#include "planning.h"
#include "myDataType.h"
#include "HexapodParameter.h"
#include "user.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "legged_traj_search/utils/gcs_visualizer.hpp"

// BUG
// IMPORTANT: Add this function to avoid Convex hull display error. (unknown reason)
void AVOID_DISPLAY_ERROR(void)
{
    Eigen::Vector3d vec(1, 1, 1);
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vec.data(), vec.cols(), false, false);
}

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
MDT::RobotState initRobotState(const MDT::Pose &robotPoseW, MDT::Vector6b gaitToNow, float moveDirection)
{
    MDT::RobotState state_;
    state_.initialize();
    state_.pose = robotPoseW;
    state_.faultStateToNow << MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG;
    for (int i = 0; i < 6; i++)
    {
        // PLANNING::POINT pnt = {HexapodParameter::transList[i].x, HexapodParameter::transList[i].y, HexapodParameter::transList[i].z};
        state_.feetPosition[i] = MDT::pointRotationAndTrans(HexapodParameter::norminalFoothold_B[i], robotPoseW.getT_W_B());
        state_.feetNormalVector[i] << 0, 0, 1; // 默认法向量竖直向上
        state_.gaitToNow[i] = gaitToNow[i];
        state_.maxNormalForce[i] = 1000.0f;
        state_.frcitionMu[i] = 0.8f;
    }
    state_.moveDirection = moveDirection;

    return state_;
}

void randomizeRobotState(MDT::RobotState &state_, double noise_amp = 0.1)
{
    for (int i = 0; i < 6; i++)
    {
        state_.feetPosition[i] += Eigen::Vector3d((rand() % 200 - 100) / 100.0 * noise_amp,
                                                  (rand() % 200 - 100) / 100.0 * noise_amp,
                                                  (rand() % 200 - 100) / 100.0 * noise_amp);
    }
    state_.pose.x += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.y += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.z += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.roll += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.pitch += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.yaw += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.moveDirection += (rand() % 200 - 100) / 100.0 * noise_amp;
}

MDT::RobotState getInitState(MDT::Pose robotPose = {-2, 0, USER::norminalTrunkHeight, 0, 0, 0 * _PI_ / 6},
                             float moveDir = 0)
{
    MDT::Vector6b gaitToNow;
    gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
    return initRobotState(robotPose, gaitToNow, moveDir);
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

class ElSpiderAirSimplePlanner
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    // HexapodSoftware Interface
    ros::Subscriber foot_state_sub_;
    legged_traj_plan::FootState foot_state_;
    bool recv_foot_state_ = false;

    // IMU
    // ros::Subscriber body_state_sub_; // not used
    // legged_traj_plan::BodyState body_state_; // not used
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped body_state_tf_;
    bool recv_body_state_ = false;

    // Interface
    std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    MCTSWholeBodyPlanner whole_body_planner_;

    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<Eigen::Vector3f> exp_path_;
    float multiply_factor_ = 0.03;
    int point_num_ = 50;

    // Misc
    SwingTrajPlannerConfig config_;

    // Status
    bool motion_lock_ = false;

    // ROS Timer event
    ros::Timer timer_;
    double init_time_ = 0.0;
    std::vector<RobotProfile> robot_profile_;

    // Visualizer
    GCSVisualizer visualizer_;
    GCSVisualizer visualizer_base_;

    // Settings
    bool fake_estimation_;
    bool fake_estimation_noisy_ = false;
    double noise_amp_ = 0.02;
    bool simulation_;

public:
    // FIXME: use ros param to init gridmap_interface_
    ElSpiderAirSimplePlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                             bool fake_estimation = false, bool simulation = false) : nh_("~"),
                                                                                      robot_interface_(std::make_shared<ElSpiderAirInterfaceROS>(nh_.param("/robot_description", std::string("")), simulation)),
                                                                                      gridmap_interface_(std::make_shared<GridMapInterface>(nh_, "/grid_map")),
                                                                                      whole_body_planner_(swing_traj_planner_config, gridmap_interface_, robot_interface_),
                                                                                      tfListener_(tfBuffer_), visualizer_(nh_, "odom", "visualizer_markers"),
                                                                                      visualizer_base_(nh_, "base", "visualizer_markers_base"),
                                                                                      rate_(100), fake_estimation_(fake_estimation), simulation_(simulation),
                                                                                      config_(swing_traj_planner_config)
    {
        init_time_ = ros::Time::now().toSec();
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirSimplePlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 1, &ElSpiderAirSimplePlanner::foot_state_callback, this);
        // body_state_sub_ = nh_.subscribe("/hexapod/body_state_fdb", 1, &ElSpiderAirSimplePlanner::body_state_callback, this);

        robot_state_.initialize();
        next_planned_state_.initialize();
        if (fake_estimation_)
        {
            robot_state_ = getInitState();
            next_planned_state_ = robot_state_;
        }
        else
        {
            timer_ = nh_.createTimer(ros::Duration(0.05), &ElSpiderAirSimplePlanner::timer_callback, this);
        }
    }

    //// Callbacks
    void timer_callback(const ros::TimerEvent &event)
    {
        pub_jointstate();
        // Update body state
        try
        {
            // Use ros::Time(0) to prevent warning of `extrapolate to future`
            body_state_tf_ = tfBuffer_.lookupTransform("odom", "base", ros::Time(0));
            recv_body_state_ = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

    // Joystick cmd callback
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        if (motion_lock_)
            ROS_WARN("Robot is in motion, ignore new command.");
        else if ((recv_foot_state_ && recv_body_state_) || fake_estimation_)
        {
            motion_lock_ = true;
            cmd_ = msg;
            whole_body_planner_.visClear();

            bool ret = false;
            // while (!ret && ros::ok())
            {
                // Fetch feedback
                ros::spinOnce();
                // update robot state
                update_exp_path();
                update_robot_state();
                // MCTS planning
                gridmap_interface_->lockMapUpdate();
                ret = CONTACT_PLANNER::pathTrackPlanner(robot_state_, next_planned_state_, exp_path_,
                                                        gridmap_interface_->getMap(), true, 600);
                // next_planned_state_ = CONTACT_PLANNER::tripleGaitPlanner(robot_state_, gridmap_interface_->getMap(), 0.1);
                gridmap_interface_->unlockMapUpdate();
            }

            // Visualization
            visualizer_.delAll();
            visualizer_base_.delAll();
            

            // Vis expected path
            if (config_.enableVis)
            {
                std::vector<Point3D> exp_path_vis;
                for (const auto &pt : exp_path_)
                {
                    exp_path_vis.emplace_back(Point3D(pt[0], pt[1], pt[2]));
                }
                visualizer_.visCurve(exp_path_vis);
            }

            // Vis getAvailableFootholds
            MDT::AvailableContactsInfo available_points = PLANNING::getAvailableFootholds_visual(next_planned_state_, gridmap_interface_->getMap());
            std::vector<Eigen::Vector3d> pts;
            for (int i = 0; i < 6; i++)
            {
                if (next_planned_state_.gaitToNow[i] == MDT::SUPPORT_FLAG)
                    continue;
                for (auto pt : available_points.position.leg[i])
                {
                    pts.emplace_back(pt);
                }
            }
            visualizer_.visSphere(pts, 0.01);

            if (ret)
            {
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(next_planned_state_));
            }
            else
            {
                ROS_INFO("MCTS failed to plan, reset to nominal state.");
                visualizer_base_.visPolytope(robot_interface_->getFootPolyhedra());
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(getInitState(robot_state_.pose, robot_state_.moveDirection)));
            }

            traj_planner();
            motion_lock_ = false;
        }
        else
        {
            ROS_WARN("No foot state feedback, ignore command.");
        }
    }

    // Foot state feedback callback
    void foot_state_callback(const legged_traj_plan::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    //// Rviz
    // Pub Real Robot JointState for Rviz
    void pub_jointstate(void)
    {
        std::vector<Eigen::Vector3d> footend_now;
        if (recv_foot_state_ && recv_body_state_)
        {
            update_robot_state();
        }
        for (size_t k = 0; k < 6; ++k)
        {
            tf2::Transform transform;
            tf2::Quaternion quaternion;
            quaternion.setRPY(robot_state_.pose.roll, robot_state_.pose.pitch, robot_state_.pose.yaw);
            transform.setOrigin(tf2::Vector3(robot_state_.pose.x, robot_state_.pose.y, robot_state_.pose.z));
            transform.setRotation(quaternion);
            Eigen::Isometry3d pose = tf2::transformToEigen(tf2::toMsg(transform));
            // inverse transform
            footend_now.emplace_back(pose.inverse() * robot_state_.feetPosition[k]);
        }
        robot_interface_->pub_joint_state_from_footendpos(footend_now);
    }

    void state_traj_replay(MCTStateTransfer &state_traj)
    {
        for (double t = 0.0; t < 1.01; t += 0.01)
        {
            // Get Interpolated State
            auto odom_interp = state_traj.eval_torso_traj(t);
            auto footend_interp = state_traj.eval_foot_traj(t); // Footend position in world frame
            auto support_state = state_traj.eval_support_state(t);
            for (size_t k = 0; k < 6; ++k)
            {
                // Convert to BASE
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }

            // Visualization
            robot_interface_->pub_odom(odom_interp, "shadowbase", "odom");
            robot_interface_->pub_shadow_joint_state_from_footendpos(footend_interp);
            if (!fake_estimation_)
            {
                ros::spinOnce();  // Fetch feedback
                pub_jointstate(); // Publish real joint state
            }

            rate_.sleep();
        }
    }

    //// MCTS Interface
    // Update Robot State for MCTS Interface
    void update_robot_state(void)
    {
        if (fake_estimation_)
        {
            robot_state_ = next_planned_state_;
            if (fake_estimation_noisy_)
            {
                randomizeRobotState(robot_state_, noise_amp_);
            }
        }
        else
        {
            // TODO: time stamp?
            robot_state_.pose.x = body_state_tf_.transform.translation.x;
            robot_state_.pose.y = body_state_tf_.transform.translation.y;
            robot_state_.pose.z = body_state_tf_.transform.translation.z;
            // RPY
            tf2::Quaternion q;
            tf2::fromMsg(body_state_tf_.transform.rotation, q);
            tf2::Matrix3x3(q).getRPY(robot_state_.pose.roll, robot_state_.pose.pitch, robot_state_.pose.yaw);

            std::vector<Eigen::Vector3d> footend_vis;
            for (int i = 0; i < 6; ++i)
            {
                // Last planned state gait (MDT::SUPPORT if not planned)
                robot_state_.gaitToNow[i] = next_planned_state_.gaitToNow[i];
                robot_state_.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                // Absolute foot position
                robot_state_.feetPosition[i] = tf2::transformToEigen(body_state_tf_.transform) *
                                               Eigen::Vector3d(foot_state_.position[i].x, foot_state_.position[i].y, foot_state_.position[i].z);
                robot_state_.feetNormalVector[i] << 0, 0, 1; // TODO: use gridmap normal
                footend_vis.emplace_back(robot_state_.feetPosition[i]);
            }

            // FIXME: cmd_ should be under robot frame
            // if (cmd_.linear.x != 0)
            //     robot_state_.moveDirection = atan2(cmd_.linear.y, cmd_.linear.x);
            robot_state_.moveDirection = robot_state_.pose.yaw;
            // TODO: maxNormalForce, frictionMu
        }
    }

    // Update Exp Path for MCTS Interface
    void update_exp_path(void)
    {
        exp_path_.clear();
        // FIXME: pose.z should be on torso height map!!! (Not used temporarily in MCTS)
        double height = 0.25;
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x, robot_state_.pose.y, height));
        for (int i = 0; i < point_num_; ++i)
        {
            exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + (cmd_.linear.x * cos(robot_state_.pose.yaw) - cmd_.linear.y * sin(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                robot_state_.pose.y + (cmd_.linear.x * sin(robot_state_.pose.yaw) + cmd_.linear.y * cos(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                height));
        }
    }

    //// Contact Handling (Sim only)
    // FIXME: avoid error detection
    bool is_contact(int leg_idx, double eps = 0.1)
    {
        Eigen::Vector3d foot_force;
        foot_force << foot_state_.effort[leg_idx].x, foot_state_.effort[leg_idx].y, foot_state_.effort[leg_idx].z;
        return foot_force.norm() > eps;
    }

    void stance_contact_handle(void)
    {
        bool flag = false;
        int max_cnt = 200;
        double alpha = 0.005;
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
            if (config_.useCfgCommand)
                robot_interface_->pub_jointcmd_from_jointpos(robot_interface_->IKFast_foots(footend_interp));
            else
                robot_interface_->pub_footcmd_from_footendpos(footend_interp);
            // FIXME: cmd should not pub at this time
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
        double t = 0.0;
        double delta = 0.005;
        MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
        pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(0.0);
        std::array<bool, 6> support_state = state_traj.eval_support_state(0.0);

        state_traj_replay(state_traj);
        if (fake_estimation_)
            ros::Duration(0.4).sleep();
        else
        {
            std::cout << "Press space to execute trajectory...";
            getchar();
        }

        do
        {
            // Get Interpolated State
            odom_interp = state_traj.eval_torso_traj(sine_remap(t));
            support_state = state_traj.eval_support_state(sine_remap(t));

            if (config_.useCfgCommand)
            {
                footend_interp = state_traj.eval_cfg_traj(sine_remap(t));
                if (fake_estimation_)
                {
                    robot_interface_->pub_joint_state(footend_interp);
                    robot_interface_->pub_odom(odom_interp);
                }
                else
                {
                    robot_interface_->pub_jointcmd_from_jointpos(footend_interp);
                    pub_jointstate();
                }
            }
            else
            {
                // Footend position in world frame
                footend_interp = state_traj.eval_foot_traj(sine_remap(t));
                for (size_t k = 0; k < 6; ++k)
                {
                    // Convert to BASE
                    footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
                }
                if (fake_estimation_)
                {
                    robot_interface_->pub_joint_state_from_footendpos(footend_interp);
                    robot_interface_->pub_odom(odom_interp);
                }
                else
                {
                    robot_interface_->pub_footcmd_from_footendpos(footend_interp);
                    pub_jointstate();
                }
            }

            if (!fake_estimation_)
            {
                ros::spinOnce();
            }

            // State recording
            RobotProfile profile;
            profile.time = ros::Time::now().toSec() - init_time_;
            profile.t = t;
            profile.pose = odom_interp;
            profile.foot_pos_list = state_traj.eval_foot_traj(t);
            profile.cfg_pos_list = state_traj.eval_cfg_traj(t, 0, false);
            profile.cfg_vel_list = state_traj.eval_cfg_traj(t, 1, false);
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
                whole_body_planner_.dequeue_MCTsolution();
                if (whole_body_planner_.get_state_traj_length() > 0)
                    state_traj = whole_body_planner_.get_state_traj(0);
            }
            rate_.sleep();
        } while (whole_body_planner_.get_state_traj_length() > 0);

        if (!fake_estimation_)
        {
            ros::spinOnce();  // Fetch feedback
            pub_jointstate(); // Publish real joint state
            ros::Duration(0.4).sleep();
            // Stance contact handling
            stance_contact_handle();
        }
    }

    void saveRobotProfile()
    {
        // Save to file
        std::ofstream file(config_.robotProfilePath);
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
    }

    void run()
    {
        // ros::spin();
        while (ros::ok())
        {
            ros::spinOnce();
        }
        if (config_.enableBenchmark)
        {
            whole_body_planner_.saveBenchmarkResults();
            saveRobotProfile();
        }
    }
};