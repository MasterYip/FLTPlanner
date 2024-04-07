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

MDT::RobotState getInitState(MDT::Pose robotPose = {1, 0, USER::norminalTrunkHeight, 0, 0, 0 * _PI_ / 6},
                             float moveDir = 0)
{
    MDT::Vector6b gaitToNow;
    gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
    return initRobotState(robotPose, gaitToNow, moveDir);
}

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

    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<Eigen::Vector3f> exp_path_;
    float multiply_factor_ = 0.03;
    int point_num_ = 3;
    // ROS Timer event
    ros::Timer timer_;
    // Visualizer
    GCSVisualizer visualizer_;

    // Interface
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    MCTSWholeBodyPlanner whole_body_planner_;
    // std::vector<hexapod_State> MCT_solution_;

    // Settings
    bool fake_estimation_;
    bool fake_estimation_noisy_ = false;
    double noise_amp_ = 0.02;
    bool simulation_;

public:
    // FIXME: use ros param to init gridmap_interface_
    ElSpiderAirSimplePlanner(bool fake_estimation = false, bool simulation = false) : nh_(), robot_interface_(nh_.param("robot_description", std::string("")), simulation),
                                                                                      gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                                                                                      tfListener_(tfBuffer_), visualizer_(nh_, "base", "visualizer_markers"),
                                                                                      rate_(20), fake_estimation_(fake_estimation), simulation_(simulation)
    {
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

    void timer_callback(const ros::TimerEvent &event)
    {
        pub_footpos_now();
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

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // Start planning
        if ((recv_foot_state_ && recv_body_state_) || fake_estimation_)
        {
            update_exp_path();
            update_robot_state();
            gridmap_interface_.lockMapUpdate();
            bool ret = CONTACT_PLANNER::pathTrackPlanner(robot_state_, next_planned_state_, exp_path_,
                                                         gridmap_interface_.getMap(), true, 100);
            gridmap_interface_.unlockMapUpdate();
            if (ret)
            {
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(next_planned_state_));
            }
            else
            {
                ROS_INFO("MCTS failed to plan, reset to nominal state.");
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(getInitState(robot_state_.pose, robot_state_.moveDirection)));
            }
            traj_planner();
        }
    }

    void foot_state_callback(const legged_traj_plan::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    void pub_footpos_now(void)
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
        robot_interface_.pub_joint_state_from_footendpos(footend_now);
    }
    // Deprecated
    [[deprecated]] void body_state_callback(const legged_traj_plan::BodyState &msg)
    {
        recv_body_state_ = true;
        // body_state_ = msg;
    }

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

            // FIXME: gaitToNow? default 0
            std::vector<Eigen::Vector3d> footend_vis;
            for (int i = 0; i < 6; ++i)
            {
                robot_state_.gaitToNow[i] = MDT::SUPPORT_FLAG; // use FootState.contact?
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

    void traj_planner()
    {
        double t = 0.0;
        double delta = 0.05;
        MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
        pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(0.0);
        do
        {
            odom_interp = state_traj.eval_torso_traj(t);
            // Footend position in world frame
            footend_interp = state_traj.eval_foot_traj(t);
            // Convert to base frame
            for (size_t k = 0; k < 6; ++k)
            {
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }
            robot_interface_.pub_footcmd_from_footendpos(footend_interp);
            if (fake_estimation_)
            {
                robot_interface_.pub_joint_state_from_footendpos(footend_interp);
                robot_interface_.pub_odom(odom_interp);
            }
            else
            {
                robot_interface_.pub_odom(odom_interp, "shadowbase", "odom");
                robot_interface_.pub_shadow_joint_state_from_footendpos(footend_interp);
                pub_footpos_now();
            }

            // Visualization
            // BUG: Sometimes the visualizer will display the convex hull as non-convex
            visualizer_.delAll();
            visualizer_.visPolytope(robot_interface_.getFootPolyhedra(2));

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
    }

    void run()
    {
        ros::spin();
    }
};