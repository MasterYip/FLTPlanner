/**
 * @file elspider_air_planner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "fast_legged_planner/elspider_air_planner/ElSpiderAirStateFollower.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_planner");
    ElSpiderAirStateFollower planner;
    planner.run();
    return 0;
}
