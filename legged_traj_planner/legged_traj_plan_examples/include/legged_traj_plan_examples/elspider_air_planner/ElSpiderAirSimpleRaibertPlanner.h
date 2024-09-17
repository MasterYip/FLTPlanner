/**
 * @file ElSpiderAirSimpleRaibertPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-18
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
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/whole_body_planner/RaibertHeuristicPlanner.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"
#include "ElSpiderAirPlannerBase.h"

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


class ElSpiderAirSimpleRaibertPlanner : public ElSpiderAirPlannerBase
{
private:
    ros::Rate rate_;
    int loop_rate_ = 500;

    // Joystick cmd subscribe
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;
    GridMapCmdVelExtrapolator cmd_extrapolator_;

    // Odometry feedback
    pinocchio::SE3 body_pose_;
    geometry_msgs::Twist body_twist_;
    geometry_msgs::Twist body_twist_base_rectify_;

    /// Interface
    SimpleRaibertPlanner whole_body_planner_;

    /// Misc
    // ROS Timer event
    ros::Timer timer_;
    int timer_loop_rate_ = 50;

    // Visualizer
    GCSVisualizer visualizer_;
    GCSVisualizer visualizer_base_;

public:
    ElSpiderAirSimpleRaibertPlanner() : ElSpiderAirPlannerBase(),
                                        whole_body_planner_(gridmap_interface_, robot_interface_),
                                        visualizer_(nh_, "odom", "visualizer_markers"),
                                        visualizer_base_(nh_, "base", "visualizer_markers_base"),
                                        rate_(loop_rate_)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirSimpleRaibertPlanner::cmd_callback, this);

        PosList pose_sample_pts;
        for (double x = -0.4; x <= 0.4; x += 0.2)
            for (double y = -0.4; y <= 0.4; y += 0.2)
                pose_sample_pts.emplace_back(Eigen::Vector3d(x, y, 0));
        cmd_extrapolator_.init(gridmap_interface_, pose_sample_pts);

        timer_ = nh_.createTimer(ros::Duration(1.0 / timer_loop_rate_), &ElSpiderAirSimpleRaibertPlanner::timer_callback, this);

        
        // Planner Init
        update_fdb();
        PosList foot_pos_list;
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        for (size_t k = 0; k < 6; ++k)
        {
            Eigen::Vector3d pos;
            pos << foot_state.position[k].x, foot_state.position[k].y, foot_state.position[k].z;
            foot_pos_list.emplace_back(point_SE3Act(body_pose_.inverse(), pos));
        }
        whole_body_planner_.start(body_pose_, foot_pos_list);

    }

    void update_fdb()
    {
        body_pose_ = robot_interface_->getBodyPoseFdb();
        // body_twist_ = robot_interface_->getBodyVelFdb();
        // body_twist_base_rectify_ = body_twist_;
        //         Eigen::Vector3d linear(msg.twist.twist.linear.x,
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
        // double scale = 0.2;
        // body_twist_base_rectify_.linear.x *= scale;
        // body_twist_base_rectify_.linear.y *= scale;
        // body_twist_base_rectify_.angular.z *= scale;
    }

    void timer_callback(const ros::TimerEvent &event)
    {
        // Update body state
        update_fdb();

        cmd_extrapolator_.update(body_pose_);
        
        // Exp body state Publish
        pinocchio::SE3 exp_pose;
        // FIXME: how to handle ref pose
        // 1. direct integration
        exp_pose = cmd_extrapolator_.extrapolate(1.0 / timer_loop_rate_);
        cmd_extrapolator_.update(exp_pose, cmd_);
        // 2. update with state
        // cmd_extrapolator_.update(body_pose_, cmd_);
        // exp_pose = cmd_extrapolator_.extrapolate(1.0 / timer_loop_rate_);

        robot_interface_->setBodyPoseCmd(exp_pose);
        robot_interface_shadow_->setBodyPoseCmd(exp_pose);

        // Exp foot state pub
        PosList exp_foot_pos;
        pinocchio::SE3 wbp_pose;
        std::array<bool, 6> contact_state;
        if (whole_body_planner_.query(ros::Time::now().toSec(), wbp_pose, exp_foot_pos, contact_state))
        {
            for (int i = 0; i < 6; ++i)
            {
                // SimpleRaibertPlanner output is in world frame, convert to BASE frame
                // FIXME: which pose to choose?
                // exp_foot_pos[i] = point_SE3Act(body_pose_, exp_foot_pos[i]);
                exp_foot_pos[i] = point_SE3Act(exp_pose, exp_foot_pos[i]);
            }
            robot_interface_->setFootCmd(exp_foot_pos,
                                         std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
                                         std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
                                         std::vector<bool>(contact_state.begin(), contact_state.end()));
            robot_interface_shadow_->setFootCmd(exp_foot_pos,
                                              std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
                                              std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
                                              std::vector<bool>(contact_state.begin(), contact_state.end()));
        }

    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // BUG: update(body_pose) is unstable
        geometry_msgs::Twist twist_mix;
        double weight = 1.0;
        twist_mix.linear.x = body_twist_base_rectify_.linear.x * (1.0 - weight) + cmd_.linear.x * weight;
        twist_mix.linear.y = body_twist_base_rectify_.linear.y * (1.0 - weight) + cmd_.linear.y * weight;
        twist_mix.linear.z = body_twist_base_rectify_.linear.z * (1.0 - weight) + cmd_.linear.z * weight;
        whole_body_planner_.update(body_pose_, twist_mix);
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
