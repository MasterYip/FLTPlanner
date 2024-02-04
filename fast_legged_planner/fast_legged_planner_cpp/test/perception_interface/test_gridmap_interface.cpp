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
#include "fast_legged_planner/perception_interface/GridmapInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_map_interface_cpp");
    GridMapInterface interface;
    ros::spin();
    return 0;
}