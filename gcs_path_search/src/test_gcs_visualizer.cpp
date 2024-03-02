/**
 * @file test_gcs_visualizer.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "misc/gcs_visualizer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_gcs_visualizer");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    GCSVisualizer gcs_visualizer(nh, "world", "test_topic");
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
        gcs_visualizer.visCurve(curve);
        gcs_visualizer.visSphere(sphere);
        gcs_visualizer.visFacet(facet);
        gcs_visualizer.visMesh(mesh);
        rate.sleep();
        gcs_visualizer.delAll();
        ros::spinOnce();
    }
}