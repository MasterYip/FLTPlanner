/**
 * @file elspider_air_swingtraj_planner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan_examples/elspider_air_planner/ElSpiderAirStateFollower.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_swingtraj_planner");
    ElSpiderAirStateFollower planner;
    planner.run();
    return 0;
}
