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

void eg_jacobian_vis(ros::NodeHandle nh)
{
    ElSpiderAirInterfaceROS robot_interface(nh.param("robot_description", std::string("")));
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(30);
    Eigen::VectorXd q(18);
    Eigen::VectorXd dq(18);
    std::vector<Eigen::Vector3d> footendpos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> footendvel(6, Eigen::Vector3d::Zero());
    
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
            for (int j=0; j<3; j++)
            {
                Eigen::Vector3d joint_pos;
                robot_interface.getRobotKin().forwardKin(q.segment<3>(3 * i), joint_pos, i, j);
                visualizer.visSphere(joint_pos, 0.13);
            }
        }

        std::vector<double> q_vec(q.data(), q.data() + q.size());
        robot_interface.pub_joint_state(q_vec);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void eg_convexhull_vis(ros::NodeHandle nh)
{
        ElSpiderAirInterfaceROS robot_interface(nh.param("robot_description", std::string("")));
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(3);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(18);
    std::vector<Eigen::Vector3d> footendpos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> footendvel(6, Eigen::Vector3d::Zero());

    Polyhedra poly = genLegPolyRegion(0);
    Eigen::Matrix3Xd hull = poly.getVRep();
    std::vector<Eigen::Matrix3Xd> sample_points;
    for (int i = 0; i < hull.cols(); i++)
    {
        sample_points.push_back(hull.col(i));
    }
    sample_points.push_back((hull.col(0) + hull.col(1) + hull.col(2) + hull.col(3)) / 4);
    sample_points.push_back((hull.col(4) + hull.col(5) + hull.col(6) + hull.col(7)) / 4);

    int vert_idx = 0;

    visualizer.visPolytope(poly);
    while (ros::ok())
    {
        // visualizer.delAll();

        // for (int i = 0; i < 18; i++)
        // {
        //     q[i] = 0.5 * sin(ros::Time::now().toSec());
        //     dq[i] = 0.5 * cos(ros::Time::now().toSec());
        // }
        q.segment<3>(3 * 0) = robot_interface.IKFast_foot(sample_points.at(vert_idx), 0);

        for (int i = 0; i < 6; i++)
        {
            robot_interface.getRobotKin().forwardKin(q.segment<3>(3 * i), footendpos[i], i);
            Eigen::Matrix3Xd J(3, 3);
            robot_interface.getRobotKin().getJacobian(q.segment<3>(3 * i), J, i);
            footendvel[i] = J * dq.segment<3>(3 * i);
            visualizer.visArrow(footendpos[i], footendpos[i] + footendvel[i] * 1);
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos;
                robot_interface.getRobotKin().forwardKin(q.segment<3>(3 * i), joint_pos, i, j);
                visualizer.visSphere(joint_pos, 0.13);
            }
        }

        if ((footendpos[0] - sample_points.at(vert_idx)).norm() > 0.01)
        {
            ROS_WARN("Vertice %d not reached!", vert_idx);
            visualizer.visSphere(sample_points.at(vert_idx), 0.01);
        }
        else
        {
            ROS_INFO("Vertice %d reached!", vert_idx);
        }
        vert_idx = (vert_idx + 1) % sample_points.size();

        std::vector<double> q_vec(q.data(), q.data() + q.size());
        robot_interface.pub_joint_state(q_vec);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robot_interface_rviz");
    ros::NodeHandle nh;
    // eg_jacobian_vis(nh);
    eg_convexhull_vis(nh);
}