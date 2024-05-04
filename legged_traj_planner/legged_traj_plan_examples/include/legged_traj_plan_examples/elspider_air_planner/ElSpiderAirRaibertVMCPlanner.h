/**
 * @file ElSpiderAirRaibertVMCPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-03
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

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
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

class ElSpiderAirRaibertVMCPlanner
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;

    // Joystick cmd subscribe
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    /// Feedback subscribe
    // Foot state feedback
    ros::Subscriber foot_state_sub_;
    legged_traj_plan::FootState foot_state_;
    bool recv_foot_state_ = false;

    /// Command publish
    ros::Publisher exp_foot_state_pub_;
    legged_traj_plan::FootState exp_foot_state_;
    ros::Publisher exp_body_state_pub_;
    nav_msgs::Odometry exp_body_state_;

    // Odometry
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped body_state_tf_;
    pinocchio::SE3 body_pose_;
    bool recv_body_state_ = false;

    /// Interface
    // Fast legged planner interface
    std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    RaibertHeuristicPlanner whole_body_planner_;
    bool planner_started_ = false;

    /// Misc
    // ROS Timer event
    ros::Timer timer_;

    // Visualizer
    GCSVisualizer visualizer_;

    // Settings
    bool fake_estimation_;
    bool fake_estimation_noisy_ = false;
    double noise_amp_ = 0.02;
    bool simulation_;

public:
    // FIXME: use ros param to init gridmap_interface_
    ElSpiderAirRaibertVMCPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                 bool fake_estimation = false, bool simulation = false) : nh_("~"),
                                                                                          robot_interface_(std::make_shared<ElSpiderAirInterfaceROS>(nh_.param("/robot_description", std::string("")), simulation)),
                                                                                          gridmap_interface_(std::make_shared<GridMapInterface>(nh_, "/grid_map")),
                                                                                          whole_body_planner_(swing_traj_planner_config, gridmap_interface_, robot_interface_),
                                                                                          tfListener_(tfBuffer_), visualizer_(nh_),
                                                                                          rate_(50), fake_estimation_(fake_estimation), simulation_(simulation)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirRaibertVMCPlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 1, &ElSpiderAirRaibertVMCPlanner::foot_state_callback, this);
        exp_foot_state_pub_ = nh_.advertise<legged_traj_plan::FootState>("/exp_foot_state", 1);
        exp_body_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/exp_odom", 1);
        exp_foot_state_.position.resize(6);
        exp_foot_state_.velocity.resize(6);
        exp_foot_state_.effort.resize(6);
        exp_foot_state_.contact.resize(6);

        timer_ = nh_.createTimer(ros::Duration(0.05), &ElSpiderAirRaibertVMCPlanner::timer_callback, this);
        if (fake_estimation_)
        {
            body_pose_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.25));
            whole_body_planner_.start(body_pose_);
            planner_started_ = true;
        }
    }

    void timer_callback(const ros::TimerEvent &event)
    {

        // Update body state (Feedback)
        if (!fake_estimation_)
        {
            try
            {
                // Use ros::Time(0) to prevent warning of `extrapolate to future`
                body_state_tf_ = tfBuffer_.lookupTransform("odom", "base", ros::Time(0));
                body_pose_.translation() = Eigen::Vector3d(body_state_tf_.transform.translation.x,
                                                           body_state_tf_.transform.translation.y,
                                                           body_state_tf_.transform.translation.z);
                body_pose_.rotation() = Eigen::Quaterniond(body_state_tf_.transform.rotation.w,
                                                           body_state_tf_.transform.rotation.x,
                                                           body_state_tf_.transform.rotation.y,
                                                           body_state_tf_.transform.rotation.z)
                                            .toRotationMatrix();
                recv_body_state_ = true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }

        if (!planner_started_)
        {
            if (recv_foot_state_ && recv_body_state_)
            {
                whole_body_planner_.start(body_pose_);
                planner_started_ = true;
            }
            else
            {
                ROS_WARN("No feedback received, planner not started");
                return;
            }
        }

        pinocchio::SE3 exp_pose;
        PosList exp_foot_pos;
        std::array<bool, 6> contact_state;
        if (whole_body_planner_.query(ros::Time::now().toSec(), exp_pose, exp_foot_pos, contact_state))
        {
            // Exp foot state
            exp_foot_state_.header.stamp = ros::Time::now();
            exp_foot_state_.header.frame_id = "odom";
            for (int i = 0; i < 6; ++i)
            {
                exp_foot_pos[i] = point_SE3Act(exp_pose, exp_foot_pos[i]);
                exp_foot_state_.position[i].x = exp_foot_pos[i](0);
                exp_foot_state_.position[i].y = exp_foot_pos[i](1);
                exp_foot_state_.position[i].z = exp_foot_pos[i](2);
                exp_foot_state_.contact[i] = contact_state[i];
            }
            exp_foot_state_pub_.publish(exp_foot_state_);

            // Exp body state
            exp_body_state_.header.stamp = ros::Time::now();
            exp_body_state_.header.frame_id = "odom";
            exp_body_state_.pose.pose.position.x = exp_pose.translation()(0);
            exp_body_state_.pose.pose.position.y = exp_pose.translation()(1);
            exp_body_state_.pose.pose.position.z = exp_pose.translation()(2);
            Eigen::Quaterniond quat(exp_pose.rotation());
            exp_body_state_.pose.pose.orientation.x = quat.x();
            exp_body_state_.pose.pose.orientation.y = quat.y();
            exp_body_state_.pose.pose.orientation.z = quat.z();
            exp_body_state_.pose.pose.orientation.w = quat.w();
            exp_body_state_pub_.publish(exp_body_state_);
        }

        if (fake_estimation_)
        {
            // pub pose tf
            robot_interface_->pub_odom(exp_pose);
            robot_interface_->pub_joint_state_from_footendpos(exp_foot_pos);
        }
    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // Start planning
        if (planner_started_)
        {
            // FIXME: update(body_pose) is unstable
            if (fake_estimation_ || recv_foot_state_ && recv_body_state_)
            {
                PosList foot_pos_list;
                std::array<bool, 6> contact_state;
                pinocchio::SE3 pose;
                whole_body_planner_.query(ros::Time::now().toSec(), pose, foot_pos_list, contact_state);
                whole_body_planner_.update(pose, cmd_);
            }
            else if (recv_foot_state_ && recv_body_state_)
            {
                std::cout << "Update planner" << std::endl;
                whole_body_planner_.update(body_pose_, cmd_);
            }
            else
            {
                ROS_WARN("No feedback received, skip planning");
            }
        }
    }

    // Feedback

    void foot_state_callback(const legged_traj_plan::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    void run(void)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            rate_.sleep();
        }
    }
};
