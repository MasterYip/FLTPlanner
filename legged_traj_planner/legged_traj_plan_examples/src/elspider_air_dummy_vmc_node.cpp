/**
 * @file elspider_air_dummy_vmc_node.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan_examples/elspider_air_planner/ElSpiderAirSimpleRaibertVMCPlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_dummy_vmc_node");
    ros::NodeHandle nh("~");
    DummyElSpiderAir robot(nh);
    robot.run();
    return 0;
}