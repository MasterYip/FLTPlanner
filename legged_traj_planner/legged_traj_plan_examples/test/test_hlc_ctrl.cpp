/**
 * @file test_hlc_ctrl.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-06-27
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

#define PI 3.1415

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_hlc_ctrl");
    ros::NodeHandle nh("~");

    ElSpiderAirInterfaceROS robot_interface(nh.param("/robot_description", std::string("")));
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(200);

    std::vector<Eigen::Vector3d> pos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> force(6, Eigen::Vector3d::Zero());

    double y_shift = 0.0;
    double z_shift = 0.0;
    std::vector<double> kp(3, 100);
    std::vector<double> kd(3, 3.0);
    std::vector<double> force_recv(3, 0.0);
    // Motion
    double interval = 1.0;
    std::vector<double> swing_amp(3, 0);
    std::vector<double> swing_phase(3, 0);

    nh.getParam("kp", kp);
    nh.getParam("kd", kd);
    nh.getParam("force", force_recv);
    nh.getParam("y_shift", y_shift);
    nh.getParam("z_shift", z_shift);
    nh.getParam("interval", interval);
    nh.getParam("swing_amp", swing_amp);
    nh.getParam("swing_phase", swing_phase);

    robot_interface.setJointKpKd(kp, kd);

    double time_start = ros::Time::now().toSec();
    while (ros::ok())
    {
        double t = ros::Time::now().toSec() - time_start;
        // Pos
        pos.at(0) << 0.354 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], -0.08 - y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];
        pos.at(1) << 0.054 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], -0.14 - y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];
        pos.at(2) << -0.354 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], -0.08 - y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];
        pos.at(3) << 0.354 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], 0.08 + y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];
        pos.at(4) << 0.054 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], 0.14 + y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];
        pos.at(5) << -0.354 + std::sin(t * 2 * PI / interval + swing_phase[0]) * swing_amp[0], 0.08 + y_shift + std::sin(t * 2 * PI / interval + swing_phase[1]) * swing_amp[1], 0.011 + z_shift + std::sin(t * 2 * PI / interval + swing_phase[2]) * swing_amp[2];

        // Force
        for (int i = 0; i < 6; i++)
            force.at(i) << force_recv[0], force_recv[1], force_recv[2];

        robot_interface.pub_footcmd_from_footendcmd(pos, vel, force);
        ros::spinOnce();
        loop_rate.sleep();
    }
}