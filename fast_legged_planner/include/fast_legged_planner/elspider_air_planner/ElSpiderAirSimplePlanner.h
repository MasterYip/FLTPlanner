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
#include "contactPlannerInterface.h"
#include "myDataType.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class ElSpiderAirPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    geometry_msgs::Twist cmd_;
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    std::vector<hexapod_State> MCT_solution_;
    ros::Rate rate_;

public:
    ElSpiderAirPlanner() : robot_interface_(nh_.param("robot_description", std::string(""))),
                           gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                           rate_(20)
    {
        sub_ = nh_.subscribe("/cmd_vel", 100, &ElSpiderAirPlanner::callback, this);
    }

    void callback(const geometry_msgs::Twist &msg)
    {
        cmd_ = msg;
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