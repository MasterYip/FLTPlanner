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
#include "legged_traj_plan/robot_interface/UnitreeA1Interface.h"
#include "legged_traj_plan/robot_interface/DummyElSpiderAirInterfaceROS.h"
// #include "ros_visualizer/ros_visualizer.hpp"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

// Forward declarations for specialized templates
void publish_joint_states_elspider(std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface, const Eigen::VectorXd &q);
void publish_joint_states_unitree(std::shared_ptr<UnitreeA1Interface> robot_interface, const Eigen::VectorXd &q, ros::Publisher &joint_pub);

// Template specialization for ElSpider
void publish_joint_states_elspider(std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface, const Eigen::VectorXd &q)
{
    std::vector<double> q_vec(q.data(), q.data() + q.size());
    robot_interface->pub_joint_state(q_vec);
}

// Template specialization for UnitreeA1
void publish_joint_states_unitree(std::shared_ptr<UnitreeA1Interface> robot_interface, const Eigen::VectorXd &q, ros::Publisher &joint_pub)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "base";
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = A1_JOINT_STATE_NAME;
    joint_state.position.resize(12, 0.0);
    joint_state.velocity.resize(12, 0.0);
    joint_state.effort.resize(12, 0.0);

    for (int i = 0; i < std::min((int)q.size(), 12); i++)
    {
        joint_state.position[i] = q[i];
    }

    joint_pub.publish(joint_state);
}

void test_elspider_jacobian_vis(std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface, ros::NodeHandle nh)
{
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(30);

    int dof = 18; // 6 legs * 3 joints
    Eigen::VectorXd q(dof);
    Eigen::VectorXd dq(dof);
    std::vector<Eigen::Vector3d> footendpos(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> footendvel(6, Eigen::Vector3d::Zero());

    while (ros::ok())
    {
        visualizer.delAll();
        for (int i = 0; i < dof; i++)
        {
            q[i] = 0.5 * sin(ros::Time::now().toSec());
            dq[i] = 0.5 * cos(ros::Time::now().toSec());
        }

        for (int i = 0; i < 6; i++)
        {
            // Use robot interface methods via getRobotKin()
            robot_interface->getRobotKin().forwardKin(q.segment<3>(3 * i), footendpos[i], i);
            Eigen::Matrix3Xd J(3, 3);
            robot_interface->getRobotKin().getJacobian(q.segment<3>(3 * i), J, i);
            footendvel[i] = J * dq.segment<3>(3 * i);

            // Visualize velocity arrows
            visualizer.visArrow(footendpos[i], footendpos[i] + footendvel[i] * 1);

            // Visualize joint positions - use default VisStyle
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos;
                robot_interface->getRobotKin().forwardKin(q.segment<3>(3 * i), joint_pos, i, j);
                visualizer.visSphere(joint_pos, 0.05);
            }
        }

        // Publish joint states
        publish_joint_states_elspider(robot_interface, q);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void test_unitree_jacobian_vis(std::shared_ptr<UnitreeA1Interface> robot_interface, ros::NodeHandle nh)
{
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(30);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    int dof = 12; // 4 legs * 3 joints
    Eigen::VectorXd q(dof);
    Eigen::VectorXd dq(dof);
    std::vector<Eigen::Vector3d> footendpos(4, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> footendvel(4, Eigen::Vector3d::Zero());

    while (ros::ok())
    {
        visualizer.delAll();
        for (int i = 0; i < dof; i++)
        {
            q[i] = 0.5 * sin(ros::Time::now().toSec());
            dq[i] = 0.5 * cos(ros::Time::now().toSec());
        }

        for (int i = 0; i < 4; i++)
        {
            // Use robot interface methods
            footendpos[i] = robot_interface->FK_foot(q.segment<3>(3 * i), i);
            Eigen::Matrix3Xd J = robot_interface->getJacobian(q.segment<3>(3 * i), i);
            footendvel[i] = J * dq.segment<3>(3 * i);

            // Visualize velocity arrows
            visualizer.visArrow(footendpos[i], footendpos[i] + footendvel[i] * 1);

            // Visualize joint positions - use default VisStyle
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos = robot_interface->FK_CollBall(q.segment<3>(3 * i), i, j);
                visualizer.visSphere(joint_pos, 0.05);
            }
        }

        // Publish joint states
        publish_joint_states_unitree(robot_interface, q, joint_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void test_elspider_convexhull_vis(std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface, ros::NodeHandle nh)
{
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(3);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(18);
    std::vector<Eigen::Vector3d> footendpos(6, Eigen::Vector3d::Zero());

    // Get polyhedron for first leg
    Polyhedra poly = genLegPolyRegion(0);
    Eigen::Matrix3Xd hull = poly.getVRep();
    std::vector<Eigen::Vector3d> sample_points;

    for (int i = 0; i < hull.cols(); i++)
    {
        sample_points.push_back(hull.col(i));
    }

    // Add center points for testing
    if (hull.cols() >= 8)
    {
        sample_points.push_back((hull.col(0) + hull.col(1) + hull.col(2) + hull.col(3)) / 4);
        sample_points.push_back((hull.col(4) + hull.col(5) + hull.col(6) + hull.col(7)) / 4);
    }

    int vert_idx = 0;
    visualizer.visPolytope(poly);

    while (ros::ok())
    {
        // Test IK for first leg with sample points
        Eigen::Vector3d q_result = robot_interface->IKFast_foot(sample_points.at(vert_idx), 0);
        q.segment<3>(0) = q_result;

        // Compute forward kinematics for all legs
        for (int i = 0; i < 6; i++)
        {
            robot_interface->getRobotKin().forwardKin(q.segment<3>(3 * i), footendpos[i], i);

            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos;
                robot_interface->getRobotKin().forwardKin(q.segment<3>(3 * i), joint_pos, i, j);
                visualizer.visSphere(joint_pos, 0.05);
            }
        }

        // Check IK accuracy
        if ((footendpos[0] - sample_points.at(vert_idx)).norm() > 0.01)
        {
            ROS_WARN("Target %d not reached! Error: %f", vert_idx, (footendpos[0] - sample_points.at(vert_idx)).norm());
            visualizer.visSphere(sample_points.at(vert_idx), 0.02);
        }
        else
        {
            ROS_INFO("Target %d reached! Error: %f", vert_idx, (footendpos[0] - sample_points.at(vert_idx)).norm());
        }

        vert_idx = (vert_idx + 1) % sample_points.size();

        // Publish joint states
        publish_joint_states_elspider(robot_interface, q);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void test_unitree_convexhull_vis(std::shared_ptr<UnitreeA1Interface> robot_interface, ros::NodeHandle nh)
{
    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(3);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(12);
    std::vector<Eigen::Vector3d> footendpos(4, Eigen::Vector3d::Zero());

    // Get polyhedron for first leg
    Polyhedra poly = robot_interface->getFootPolyhedra(0);
    Eigen::Matrix3Xd hull = poly.getVRep();
    std::vector<Eigen::Vector3d> sample_points;

    for (int i = 0; i < hull.cols(); i++)
    {
        sample_points.push_back(hull.col(i));
    }

    // Add center points for testing
    if (hull.cols() >= 8)
    {
        sample_points.push_back((hull.col(0) + hull.col(1) + hull.col(2) + hull.col(3)) / 4);
        sample_points.push_back((hull.col(4) + hull.col(5) + hull.col(6) + hull.col(7)) / 4);
    }

    int vert_idx = 0;
    visualizer.visPolytope(poly);

    while (ros::ok())
    {
        // Test IK for first leg with sample points
        Eigen::Vector3d q_result;
        bool ik_success = robot_interface->IKFast_foot(sample_points.at(vert_idx), q_result, 0);

        if (ik_success)
        {
            q.segment<3>(0) = q_result;
        }
        else
        {
            ROS_WARN("IK failed for point %d, using nominal position", vert_idx);
            q.segment<3>(0) = robot_interface->IKFast_foot(robot_interface->getNominalFoothold(0), 0);
        }

        // Compute forward kinematics for all legs
        for (int i = 0; i < 4; i++)
        {
            footendpos[i] = robot_interface->FK_foot(q.segment<3>(3 * i), i);

            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos = robot_interface->FK_CollBall(q.segment<3>(3 * i), i, j);
                visualizer.visSphere(joint_pos, 0.05);
            }
        }

        // Check IK accuracy
        if ((footendpos[0] - sample_points.at(vert_idx)).norm() > 0.01)
        {
            ROS_WARN("Target %d not reached! Error: %f", vert_idx, (footendpos[0] - sample_points.at(vert_idx)).norm());
            visualizer.visSphere(sample_points.at(vert_idx), 0.02);
        }
        else
        {
            ROS_INFO("Target %d reached! Error: %f", vert_idx, (footendpos[0] - sample_points.at(vert_idx)).norm());
        }

        vert_idx = (vert_idx + 1) % sample_points.size();

        // Publish joint states
        publish_joint_states_unitree(robot_interface, q, joint_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void test_unitree_a1_fk_ik(ros::NodeHandle nh)
{
    ROS_INFO("Testing UnitreeA1 FK & IK");

    // Create UnitreeA1 interface with dummy robot description for visualization
    std::string urdf_param = nh.param("/robot_description", std::string(""));
    auto robot_interface = std::make_shared<UnitreeA1Interface>(urdf_param);

    GCSVisualizer visualizer(nh, std::string("base"), std::string("visualizer_markers"));
    ros::Rate loop_rate(5);

    // Test points in base frame
    std::vector<Eigen::Vector3d> test_points = {
        robot_interface->getNominalFoothold(0), // FR nominal
        robot_interface->getNominalFoothold(1), // FL nominal
        robot_interface->getNominalFoothold(2), // RR nominal
        robot_interface->getNominalFoothold(3), // RL nominal
        Eigen::Vector3d(0.15, -0.10, -0.25),    // Custom test point 1
        Eigen::Vector3d(0.20, 0.15, -0.35),     // Custom test point 2
        Eigen::Vector3d(-0.15, -0.10, -0.30),   // Custom test point 3
        Eigen::Vector3d(-0.20, 0.15, -0.28)     // Custom test point 4
    };

    int point_idx = 0;
    int leg_idx = 0;

    // Publish dummy joint states for visualization
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "base";
    joint_state.name = A1_JOINT_STATE_NAME;
    joint_state.position.resize(12, 0.0);
    joint_state.velocity.resize(12, 0.0);
    joint_state.effort.resize(12, 0.0);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    while (ros::ok())
    {
        visualizer.delAll();

        Eigen::Vector3d target_point = test_points[point_idx];

        // Test IK with constraint checking
        Eigen::Vector3d q_result;
        bool ik_success = robot_interface->IKFast_foot(target_point, q_result, leg_idx);

        if (ik_success)
        {
            // Test FK to verify IK result
            Eigen::Vector3d fk_result = robot_interface->FK_foot(q_result, leg_idx);
            double error = (fk_result - target_point).norm();

            ROS_INFO("Leg %d, Point %d: IK Success, FK Error: %f mm",
                     leg_idx, point_idx, error * 1000);

            // Visualize target point (red) - use VisStyle constructor
            ros_visualizer::VisStyle red_style(1.0, 0.0, 0.0, 1.0, 0.03);
            visualizer.visSphere(target_point, 0.03, red_style);

            // Visualize FK result (green if accurate, yellow if error)
            ros_visualizer::VisStyle result_style;
            if (error < 0.001)
            {
                result_style = ros_visualizer::VisStyle(0.0, 1.0, 0.0, 1.0, 0.025); // green
            }
            else
            {
                result_style = ros_visualizer::VisStyle(1.0, 1.0, 0.0, 1.0, 0.025); // yellow
            }
            visualizer.visSphere(fk_result, 0.025, result_style);

            // Visualize joint positions (blue)
            ros_visualizer::VisStyle blue_style(0.0, 0.0, 1.0, 1.0, 0.015);
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d joint_pos = robot_interface->FK_CollBall(q_result, leg_idx, j);
                visualizer.visSphere(joint_pos, 0.015, blue_style);
            }

            // Test Jacobian
            Eigen::Matrix3Xd J = robot_interface->getJacobian(q_result, leg_idx);
            Eigen::Vector3d test_vel(0.1, 0.1, 0.1);
            Eigen::Vector3d foot_vel = J * test_vel;
            visualizer.visArrow(fk_result, fk_result + foot_vel * 0.5);

            // Update joint state for visualization
            for (int i = 0; i < 3; i++)
            {
                joint_state.position[leg_idx * 3 + i] = q_result[i];
            }
        }
        else
        {
            ROS_WARN("Leg %d, Point %d: IK Failed - point unreachable", leg_idx, point_idx);
            ros_visualizer::VisStyle red_style(1.0, 0.0, 0.0, 1.0, 0.03);
            visualizer.visSphere(target_point, 0.03, red_style); // red for failed points
        }

        // Publish joint states
        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);

        // Move to next test case
        point_idx = (point_idx + 1) % test_points.size();
        if (point_idx == 0)
        {
            leg_idx = (leg_idx + 1) % 4;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void eg_jacobian_vis(ros::NodeHandle nh)
{
    ElSpiderAirInterfaceROS robot_interface(nh.param("/robot_description", std::string("")));
    auto shared_interface = std::make_shared<ElSpiderAirInterfaceROS>(robot_interface);
    test_elspider_jacobian_vis(shared_interface, nh);
}

void eg_convexhull_vis(ros::NodeHandle nh)
{
    ElSpiderAirInterfaceROS robot_interface(nh.param("/robot_description", std::string("")));
    auto shared_interface = std::make_shared<ElSpiderAirInterfaceROS>(robot_interface);
    test_elspider_convexhull_vis(shared_interface, nh);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robot_interface_rviz");
    ros::NodeHandle nh("~");

    // Get robot type parameter
    std::string robot_type = nh.param("robot_type", std::string("elspider"));
    std::string test_type = nh.param("test_type", std::string("jacobian"));

    ROS_INFO("Testing robot type: %s, test type: %s", robot_type.c_str(), test_type.c_str());

    if (robot_type == "unitree_a1")
    {
        std::string urdf_param = nh.param("/robot_description", std::string(""));
        auto robot_interface = std::make_shared<UnitreeA1Interface>(urdf_param);

        if (test_type == "fk_ik")
        {
            test_unitree_a1_fk_ik(nh);
        }
        else if (test_type == "jacobian")
        {
            test_unitree_jacobian_vis(robot_interface, nh);
        }
        else if (test_type == "convexhull")
        {
            test_unitree_convexhull_vis(robot_interface, nh);
        }
    }
    else if (robot_type == "elspider")
    {
        if (test_type == "jacobian")
        {
            eg_jacobian_vis(nh);
        }
        else if (test_type == "convexhull")
        {
            eg_convexhull_vis(nh);
        }
    }
    else
    {
        ROS_ERROR("Unknown robot type: %s. Supported: elspider, unitree_a1", robot_type.c_str());
        return -1;
    }

    return 0;
}