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

#include "fast_legged_planner/elspider_air_planner/ElSpiderAirSimplePlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_simple_planner");
    ros::Time::init(); // FIXME: some where call ros::Time::now() before nh_ initialized
    bool fake_feedback = ros::param::param<bool>("~fake_feedback", false);
    bool simulation = ros::param::param<bool>("~sim", false);
    ElSpiderAirSimplePlanner planner(fake_feedback, simulation);
    planner.run();
    return 0;
}