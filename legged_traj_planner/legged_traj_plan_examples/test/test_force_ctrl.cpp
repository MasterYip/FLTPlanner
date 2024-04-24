/**
 * @file test_force_ctrl.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <vector>
#include <string>
#include <math.h>
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_force_ctrl");
    ros::NodeHandle nh("~");

    ElSpiderAirInterfaceROS robot_interface(nh.param("/robot_description", std::string("")));
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(200);

    std::vector<Eigen::Vector3d> pos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> force(6, Eigen::Vector3d::Zero());

    double y_shift = 0.0;
    double z_shift = 0.0;
    nh.getParam("y_shift", y_shift);
    nh.getParam("z_shift", z_shift);
    pos.at(0) << 0.354, -0.08 - y_shift, 0.011 + z_shift;
    pos.at(1) << 0.054, -0.14 - y_shift, 0.011 + z_shift;
    pos.at(2) << -0.354, -0.08 - y_shift, 0.011 + z_shift;
    pos.at(3) << 0.354, 0.08 + y_shift, 0.011 + z_shift;
    pos.at(4) << 0.054, 0.14 + y_shift, 0.011 + z_shift;
    pos.at(5) << -0.354, 0.08 + y_shift, 0.011 + z_shift;

    std::vector<double> kp(3, 100);
    std::vector<double> kd(3, 3.0);
    nh.getParam("kp", kp);
    nh.getParam("kd", kd);
    robot_interface.setJointKpKd(kp, kd);

    std::vector<double> force_recv(3, 0.0);
    nh.getParam("force", force_recv);
    for (int i = 0; i < 6; i++)
        force.at(i) << force_recv[0], force_recv[1], force_recv[2];
    while (ros::ok())
    {
        robot_interface.pub_footcmd_from_footendcmd(pos, vel, force);
        ros::spinOnce();
        loop_rate.sleep();
    }
}