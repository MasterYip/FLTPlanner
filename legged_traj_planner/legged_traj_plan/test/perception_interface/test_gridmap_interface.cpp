/**
 * @file test_gridmap_interface.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "legged_traj_plan/perception_interface/GridMapInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_gridmap_interface");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    GridMapInterface interface(nh, "/grid_map");
    grid_map::Length range = interface.getRange();
    std::cout << "Range: " << range[0] << " " << range[1] << std::endl;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> sdf_range = interface.getSdfRange();
    std::cout << "SDF Range: " << std::endl
              << std::setprecision(4)
              << sdf_range.first.transpose() << std::endl
              << sdf_range.second.transpose() << std::endl;
    while (ros::ok())
    {
        grid_map::Position pos(range[0] / 2, range[1] / 2);
        double height = interface.value(pos);
        std::cout << "Height at center: " << height << std::endl;
        grid_map::Position3 pos3(pos.x(), pos.y(), height);
        double sdf_value = interface.sdfValue(pos3);
        std::cout << "SDF at center: " << sdf_value << std::endl;
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}