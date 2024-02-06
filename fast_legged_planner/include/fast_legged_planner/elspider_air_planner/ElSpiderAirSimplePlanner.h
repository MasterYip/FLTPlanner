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
#include "fast_legged_planner/robot_interface/ElSpiderAirInterfaceROS.h" // Should be included first (pinocchio)
#include "fast_legged_planner/swing_leg_planner/SwingTrajPlanner.h"
#include "fast_legged_planner/perception_interface/GridMapInterface.h"
#include "fast_legged_planner/whole_body_planner/WholeBodyPlanner.h"

#include "fast_legged_planner/hexapod_State.h"
#include "fast_legged_planner/FootState.h"
#include "fast_legged_planner/BodyState.h"
// MCTS
#include "contactPlannerInterface.h"
#include "myDataType.h"
#include "HexapodParameter.h"
#include "user.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

fast_legged_planner::hexapod_State transRobotState(const MDT::RobotState &state_)
{
    fast_legged_planner::hexapod_State hexapodState;
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
    fast_legged_planner::FootState foot_state_;
    bool recv_foot_state_ = false;
    ros::Subscriber body_state_sub_;
    fast_legged_planner::BodyState body_state_;
    bool recv_body_state_ = false;
    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<Eigen::Vector3f> exp_path_;
    float multiply_factor_ = 0.2;

    // Interface
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    // std::vector<hexapod_State> MCT_solution_;

    // Settings
    bool fake_estimation_;

public:
    ElSpiderAirSimplePlanner(bool fake_estimation = false) : nh_(),robot_interface_(nh_.param("robot_description", std::string(""))),
                                                             gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                                                             rate_(20), fake_estimation_(fake_estimation)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 100, &ElSpiderAirSimplePlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 100, &ElSpiderAirSimplePlanner::foot_state_callback, this);
        body_state_sub_ = nh_.subscribe("/hexapod/body_state_fdb", 100, &ElSpiderAirSimplePlanner::body_state_callback, this);

        robot_state_.initialize();
        next_planned_state_.initialize();
        if (fake_estimation_)
        {
            // 初始化机器人状态,并赋初值
            MDT::Pose robotPoseW = {1, 0, USER::norminalTrunkHeight, 0, 0, 0 * _PI_ / 6};
            MDT::Vector6b gaitToNow;
            gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
            float moveDir = 0 * _PI_ / 2;
            robot_state_ = initRobotState(robotPoseW, gaitToNow, moveDir);
        }
    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        cmd_ = msg;
        // Start planning
        if ((recv_foot_state_ && recv_body_state_) || fake_estimation_)
        {
            update_robot_state();
            update_exp_path();
            next_planned_state_ = CONTACT_PLANNER::pathTrackPlanner(robot_state_, exp_path_, gridmap_interface_.getMap(), true);
            whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_), transRobotState(next_planned_state_));
            traj_planner();
        }
    }

    void foot_state_callback(const fast_legged_planner::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    void body_state_callback(const fast_legged_planner::BodyState &msg)
    {
        recv_body_state_ = true;
        body_state_ = msg;
    }

    void update_robot_state(void)
    {
        if (fake_estimation_)
        {
            robot_state_ = next_planned_state_;
        }
        else
        {
            // TODO: time stamp?
            robot_state_.pose.x = body_state_.pose.position.x;
            robot_state_.pose.y = body_state_.pose.position.y;
            robot_state_.pose.z = body_state_.pose.position.z;
            robot_state_.pose.roll = body_state_.eular.roll;
            robot_state_.pose.pitch = body_state_.eular.pitch;
            robot_state_.pose.yaw = body_state_.eular.yaw;
            // FIXME: gaitToNow? default 0
            for (int i = 0; i < 6; ++i)
            {
                robot_state_.gaitToNow[i] = MDT::SUPPORT_FLAG; // use FootState.contact?
                robot_state_.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                robot_state_.feetPosition[i].x() = foot_state_.position[i].x;
                robot_state_.feetPosition[i].y() = foot_state_.position[i].y;
                robot_state_.feetPosition[i].z() = foot_state_.position[i].z;
                robot_state_.feetNormalVector[i] << 0, 0, 1; // TODO: use gridmap normal
            }
            // FIXME: cmd_ should be under robot frame
            if (cmd_.linear.x != 0)
                robot_state_.moveDirection = atan2(cmd_.linear.y, cmd_.linear.x);
            // TODO: maxNormalForce, frictionMu
        }
    }

    void update_exp_path(void)
    {
        exp_path_.clear();
        // FIXME: pose.z is const?
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x, robot_state_.pose.y, robot_state_.pose.z));
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + cmd_.linear.x * multiply_factor_,
                                            robot_state_.pose.y + cmd_.linear.y * multiply_factor_,
                                            robot_state_.pose.z + cmd_.linear.z));
    }

    void traj_planner()
    {
        double t = 0.0;
        double delta = 0.05;
        while (whole_body_planner_.get_state_traj_length() > 0)
        {
            MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
            pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(t);
            std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(t);
            for (size_t k = 0; k < 6; ++k)
            {
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }
            robot_interface_.pub_footcmd_from_footendpos(footend_interp);
            robot_interface_.pub_joint_state_from_footendpos(footend_interp);
            robot_interface_.pub_odom(odom_interp);
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                whole_body_planner_.dequeue_MCTsolution();
            }
            rate_.sleep();
        }
    }

    void run()
    {
        ros::spin();
    }
};