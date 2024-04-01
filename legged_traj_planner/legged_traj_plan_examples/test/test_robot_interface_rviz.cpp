/**
 * @file test_robot_interface_rviz.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <vector>
#include <string>
#include <math.h>
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h"
// #include "ros_visualizer/ros_visualizer.hpp"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robot_interface_rviz");
    ros::NodeHandle nh;
    ElSpiderAirInterfaceROS robot_interface(nh.param("robot_description", std::string("")));
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(30);
    Eigen::VectorXd q(18);
    Eigen::VectorXd dq(18);
    std::vector<Eigen::Vector3d> footendpos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> footendvel(6, Eigen::Vector3d::Zero());
    Eigen::Matrix3Xd FootHull(3, 10);
    FootHull << 0.2412, -0.07939, -0.0809, 0.2556, -0.3199, -0.2209, 0.3721, 0.3527, 0.05979, 0.06059,
        -0.154, -0.1551, -0.1567, -0.1674, -0.3958, -0.2967, -0.2772, -0.2589, -0.4186, -0.472,
        -0.1303, -0.1464, -0.3889, -0.3545, 0.006312, -0.3344, 0.02371, -0.2644, -0.2857, 0.1195;
    
    while (ros::ok())
    {
        visualizer.delAll();
        for (int i = 0; i < 18; i++)
        {
            q[i] = 0.5 * sin(ros::Time::now().toSec());
            dq[i] = 0.5 * cos(ros::Time::now().toSec());
        }
        for (int i = 0; i < 6; i++)
        {
            robot_interface.getRobotKin().forwardKin(q.segment<3>(3 * i), footendpos[i], i);
            Eigen::Matrix3Xd J(3, 3);
            robot_interface.getRobotKin().getJacobian(q.segment<3>(3 * i), J, i);
            footendvel[i] = J * dq.segment<3>(3 * i);
            visualizer.visArrow(footendpos[i], footendpos[i] + footendvel[i] * 1);
        }
        visualizer.visPolytope(FootHull);

        std::vector<double> q_vec(q.data(), q.data() + q.size());
        robot_interface.pub_joint_state(q_vec);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}