/**
 * @file test_raibert_planner_rviz.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <vector>
#include <string>
#include <math.h>
#include "legged_traj_plan/whole_body_planner/WholeBodyPlanner.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void eg_cmdvel_extrapolator(ros::NodeHandle &nh)
{
    ros::Rate rate(10);
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    CmdVelExtrapolator extrapolator;
    pinocchio::SE3 pose = pinocchio::SE3::Identity();
    pinocchio::SE3 pose_tmp;
    geometry_msgs::Twist cmd_vel;
    while (ros::ok())
    {
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.1;
        cmd_vel.angular.y = 0.1;
        extrapolator.update(pose, cmd_vel);

        // Visualize
        visualizer.delAll();
        for (double t = 0; t < 5; t += 0.1)
        {
            pose_tmp = extrapolator.extrapolate(t);
            Eigen::Vector4d quat_vec;
            pinocchio::SE3::Quaternion quat(pose_tmp.rotation()); // This is xyzw
            quat_vec << quat.coeffs().w(), quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z();
            visualizer.visCube(pose_tmp.translation(), quat_vec);
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_raibert_planner_rviz");
    ros::NodeHandle nh;
    eg_cmdvel_extrapolator(nh);
}