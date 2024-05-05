/**
 * @file ElSpiderAirSimpleRaibertVMCPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief Simple raibert planner for EiSpiderAir (instant trajectory planning)
 * @version 0.1
 * @date 2024-05-05
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

class ElSpiderAirSimpleRaibertVMCPlanner
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;

    // Joystick cmd subscribe
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;
    GridMapCmdVelExtrapolator cmd_extrapolator_;
    double cmd_extrapolate_time_ = 0.5;

    /// Feedback subscribe
    // Foot state feedback
    ros::Subscriber foot_state_sub_;
    legged_traj_plan::FootState foot_state_;
    PosList foot_pos_list_;
    bool recv_foot_state_ = false;

    // Odometry feedback
    ros::Subscriber body_state_sub_;
    pinocchio::SE3 body_pose_;
    geometry_msgs::Twist body_twist_;
    geometry_msgs::Twist body_twist_base_rectify_;
    bool recv_body_state_ = false;

    /// Command publish
    ros::Publisher exp_foot_state_pub_;
    legged_traj_plan::FootState exp_foot_state_;
    ros::Publisher exp_body_state_pub_;
    nav_msgs::Odometry exp_body_state_;

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
    ElSpiderAirSimpleRaibertVMCPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                       bool fake_estimation = false, bool simulation = false) : nh_("~"),
                                                                                                robot_interface_(std::make_shared<ElSpiderAirInterfaceROS>(nh_.param("/robot_description", std::string("")), simulation)),
                                                                                                gridmap_interface_(std::make_shared<GridMapInterface>(nh_, "/grid_map")),
                                                                                                whole_body_planner_(swing_traj_planner_config, gridmap_interface_, robot_interface_),
                                                                                                visualizer_(nh_, "base", "visualizer_marker"),
                                                                                                rate_(50), fake_estimation_(fake_estimation), simulation_(simulation)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirSimpleRaibertVMCPlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 1, &ElSpiderAirSimpleRaibertVMCPlanner::foot_state_callback, this);
        body_state_sub_ = nh_.subscribe("/base_odom", 1, &ElSpiderAirSimpleRaibertVMCPlanner::body_state_callback, this);

        exp_foot_state_pub_ = nh_.advertise<legged_traj_plan::FootState>("/exp_foot_state", 1);
        exp_body_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/exp_odom", 1);
        exp_foot_state_.position.resize(6);
        exp_foot_state_.velocity.resize(6);
        exp_foot_state_.effort.resize(6);
        exp_foot_state_.contact.resize(6);

        PosList pose_sample_pts;
        for (double x = -0.4; x <= 0.4; x += 0.2)
        {
            for (double y = -0.4; y <= 0.4; y += 0.2)
            {
                pose_sample_pts.emplace_back(Eigen::Vector3d(x, y, 0));
            }
        }
        cmd_extrapolator_.init(gridmap_interface_, pose_sample_pts);

        timer_ = nh_.createTimer(ros::Duration(0.05), &ElSpiderAirSimpleRaibertVMCPlanner::timer_callback, this);
        if (fake_estimation_)
        {
            body_pose_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.25));
            whole_body_planner_.start(body_pose_);
            planner_started_ = true;
        }
    }

    void timer_callback(const ros::TimerEvent &event)
    {

        if (!planner_started_)
        {
            if (recv_foot_state_ && recv_body_state_)
            {
                whole_body_planner_.start(body_pose_, foot_pos_list_);
                cmd_extrapolator_.update(body_pose_);
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
                exp_foot_state_.position[i].x = exp_foot_pos[i](0);
                exp_foot_state_.position[i].y = exp_foot_pos[i](1);
                exp_foot_state_.position[i].z = exp_foot_pos[i](2);
                exp_foot_state_.contact[i] = contact_state[i];
            }
            exp_foot_state_pub_.publish(exp_foot_state_);

            // FIXME: how to handle ref pose
            // Exp body state
            exp_pose = cmd_extrapolator_.extrapolate(cmd_extrapolate_time_);
            cmd_extrapolator_.update(exp_pose, cmd_);

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
            geometry_msgs::Twist twist_world;
            Eigen::Vector3d linear_world;
            linear_world << cmd_.linear.x, cmd_.linear.y, cmd_.linear.z;
            linear_world = exp_pose.rotation() * linear_world;
            Eigen::Vector3d angular_world;
            angular_world << cmd_.angular.x, cmd_.angular.y, cmd_.angular.z;
            angular_world = exp_pose.rotation() * angular_world;
            twist_world.linear.x = linear_world(0);
            twist_world.linear.y = linear_world(1);
            twist_world.linear.z = linear_world(2);
            twist_world.angular.x = angular_world(0);
            twist_world.angular.y = angular_world(1);
            twist_world.angular.z = angular_world(2);
            exp_body_state_.twist.twist = twist_world;
            exp_body_state_pub_.publish(exp_body_state_);
        }

        if (fake_estimation_)
        {
            // pub pose tf
            robot_interface_->pub_odom(exp_pose);
            robot_interface_->pub_joint_state_from_footendpos(exp_foot_pos);
        }
        else
        {
            // Shadow robot
            robot_interface_->pub_odom(exp_pose, "shadowbase", "odom");
            robot_interface_->pub_shadow_joint_state_from_footendpos(exp_foot_pos);
            pub_footpos_now();
        }
    }

    void pub_footpos_now(void)
    {
        std::vector<Eigen::Vector3d> footend_now;
        for (size_t k = 0; k < 6; ++k)
        {
            Eigen::Vector3d pos;
            pos << foot_state_.position[k].x, foot_state_.position[k].y, foot_state_.position[k].z;
            footend_now.emplace_back(pos);
        }
        robot_interface_->pub_joint_state_from_footendpos(footend_now);
    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // Start planning
        if (planner_started_)
        {
            if (fake_estimation_)
            {
                PosList foot_pos_list;
                std::array<bool, 6> contact_state;
                pinocchio::SE3 pose;
                whole_body_planner_.query(ros::Time::now().toSec(), pose, foot_pos_list, contact_state);
                whole_body_planner_.update(pose, cmd_);
            }
            else if (recv_foot_state_ && recv_body_state_)
            {
                // BUG: update(body_pose) is unstable 
                whole_body_planner_.update(body_pose_, body_twist_base_rectify_);
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
        foot_pos_list_.clear();
        for (size_t k = 0; k < 6; ++k)
        {
            Eigen::Vector3d pos;
            pos << foot_state_.position[k].x, foot_state_.position[k].y, foot_state_.position[k].z;
            foot_pos_list_.emplace_back(point_SE3Act(body_pose_.inverse(), pos));
        }
    }

    void body_state_callback(const nav_msgs::Odometry &msg)
    {
        recv_body_state_ = true;
        body_pose_.translation() = Eigen::Vector3d(msg.pose.pose.position.x,
                                                   msg.pose.pose.position.y,
                                                   msg.pose.pose.position.z);
        Eigen::Quaterniond quat(msg.pose.pose.orientation.w,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z);
        body_pose_.rotation() = quat.toRotationMatrix();
        body_twist_ = msg.twist.twist; // FIXME: seems msg.twist is in base frame
        body_twist_base_rectify_ = msg.twist.twist;
        body_twist_base_rectify_.linear.z = 0;
        body_twist_base_rectify_.angular.x = 0;
        body_twist_base_rectify_.angular.y = 0;

        // Eigen::Vector3d linear(msg.twist.twist.linear.x,
        //                        msg.twist.twist.linear.y,
        //                        msg.twist.twist.linear.z);
        // linear = body_pose_.rotation().inverse() * linear;
        // body_twist_base_rectify_.linear.x = linear(0);
        // body_twist_base_rectify_.linear.y = linear(1);
        // body_twist_base_rectify_.linear.z = 0;
        // Eigen::Vector3d angular(msg.twist.twist.angular.x,
        //                         msg.twist.twist.angular.y,
        //                         msg.twist.twist.angular.z);
        // angular = body_pose_.rotation().inverse() * angular;
        // body_twist_base_rectify_.angular.x = 0;
        // body_twist_base_rectify_.angular.y = 0;
        // body_twist_base_rectify_.angular.z = angular(2);

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
