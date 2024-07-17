/**
 * @file ros_visualizer_test_node.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ros_visualizer/ros_visualizer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_visualizer_test_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    ros_visualizer::ROSVisualizer rv(nh);
    std::vector<Eigen::Vector3d> curve;
    curve.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    curve.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
    curve.emplace_back(Eigen::Vector3d(2.0, 2.0, 2.0));
    std::vector<Eigen::Vector3d> sphere;
    sphere.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    sphere.emplace_back(Eigen::Vector3d(1.0, 1.0, 1.0));
    sphere.emplace_back(Eigen::Vector3d(2.0, 2.0, 2.0));
    Eigen::MatrixX3d facet;
    facet.resize(6, 3);
    facet << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        5.0, 0.0, 0.0,
        0.0, 5.0, 0.0,
        0.0, 0.0, 5.0;
    Eigen::MatrixX3d mesh;
    mesh.resize(6, 3);
    mesh << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0;
    while (ros::ok())
    {
        // rv.visCurve(curve, ros_visualizer::VisStyle(1.0, 0.0, 0.0, 1.0, 0.05));
        // rv.visSphere(sphere, ros_visualizer::VisStyle(0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0));
        rv.visCurve(curve);
        rv.visSphere(sphere);
        rv.visFacet(facet);
        rv.visMesh(mesh);
        rate.sleep();
        // rv.delCurve();
        // rate.sleep();
        rv.delAll();
        ros::spinOnce();
    }
    return 0;
}
