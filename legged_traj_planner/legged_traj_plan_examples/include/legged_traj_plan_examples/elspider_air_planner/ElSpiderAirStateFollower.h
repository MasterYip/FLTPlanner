/**
 * @file ElSpiderAirStateFollower.h
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

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class ElSpiderAirStateFollower
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    std::vector<hexapod_State> MCT_solution_;
    ros::Rate rate_;

public:
    ElSpiderAirStateFollower() : robot_interface_(nh_.param("robot_description", std::string(""))),
                                 gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                                 rate_(20)
    {
        sub_ = nh_.subscribe("/supportStateTopic", 100, &ElSpiderAirStateFollower::callback, this);
    }

    void callback(const legged_traj_plan::hexapod_State &msg)
    {
        MCT_solution_.push_back(msg);
        if (msg.remarks.data == "end_flag")
        {
            ROS_INFO("end_flag received, start planning");
            for (size_t i = 0; i < MCT_solution_.size() - 1; ++i)
            {
                hexapod_State state_0 = MCT_solution_[i];
                hexapod_State state_1 = MCT_solution_[i + 1];
                whole_body_planner_.enqueue_MCTsolution(state_0, state_1);
            }
            MCT_solution_.clear();
            traj_planner();
        }
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