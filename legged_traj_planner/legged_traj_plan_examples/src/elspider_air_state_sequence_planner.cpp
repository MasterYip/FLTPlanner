/**
 * @file elspider_air_simple_planner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan_examples/elspider_air_planner/ElSpiderAirStateSequencePlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_state_sequence_planner");
    // Create planner
    ElSpiderAirStateSequencePlanner planner;
    planner.run();
    return 0;
}