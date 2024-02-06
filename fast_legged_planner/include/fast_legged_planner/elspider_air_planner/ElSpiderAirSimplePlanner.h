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

#include "contactPlannerInterface.h"
#include "myDataType.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class ElSpiderAirPlanner
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
    ros::Subscriber body_state_sub_;
    fast_legged_planner::BodyState body_state_;
    // MCTS planner Interface
    MDT::RobotState robot_state_;

    // Interface
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    // std::vector<hexapod_State> MCT_solution_;

public:
    ElSpiderAirPlanner() : robot_interface_(nh_.param("robot_description", std::string(""))),
                           gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                           rate_(20)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 100, &ElSpiderAirPlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 100, &ElSpiderAirPlanner::foot_state_callback, this);
        body_state_sub_ = nh_.subscribe("/hexapod/body_state_fdb", 100, &ElSpiderAirPlanner::body_state_callback, this);

        robot_state_.initialize();
    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        cmd_ = msg;
    }

    void foot_state_callback(const fast_legged_planner::FootState &msg)
    {
        foot_state_ = msg;
    }

    void body_state_callback(const fast_legged_planner::BodyState &msg)
    {
        body_state_ = msg;
    }

    void update_robot_state(void)
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
        if (cmd_.linear.x != 0)
            robot_state_.moveDirection = atan2(cmd_.linear.y, cmd_.linear.x);
        // TODO: maxNormalForce, frictionMu
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