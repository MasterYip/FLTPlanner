/**
 * @file test_visualizer.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <vector>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include "legged_traj_search/utils/gcs_visualizer.hpp"

// BUG
// IMPORTANT: Add this function to avoid Convex hull display error. (unknown reason)
void AVOID_DISPLAY_ERROR(void)
{
    Eigen::Vector3d vec(1, 1, 1);
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vec.data(), vec.cols(), false, false);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_visualizer");
    ros::NodeHandle nh;
    GCSVisualizer visualizer(nh, std::string("odom"), std::string("visualizer_markers"));
    ros::Rate loop_rate(100);

    Eigen::Matrix3Xd FootHull(3, 10);
    FootHull << 0.2412, -0.07939, -0.0809, 0.2556, -0.3199, -0.2209, 0.3721, 0.3527, 0.05979, 0.06059,
        -0.154, -0.1551, -0.1567, -0.1674, -0.3958, -0.2967, -0.2772, -0.2589, -0.4186, -0.472,
        -0.1303, -0.1464, -0.3889, -0.3545, 0.006312, -0.3344, 0.02371, -0.2644, -0.2857, 0.1195;

    while (ros::ok())
    {
        visualizer.delAll();
        visualizer.visPolytope(FootHull);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}