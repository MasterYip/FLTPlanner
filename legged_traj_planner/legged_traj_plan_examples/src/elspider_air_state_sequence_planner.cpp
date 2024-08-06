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
    ros::NodeHandle nh("~");
    ros::Time::init(); // FIXME: some where call ros::Time::now() before nh_ initialized
    bool fake_feedback = ros::param::param<bool>("~fake_feedback", false);
    bool simulation = ros::param::param<bool>("~sim", false);
    SwingTrajPlannerConfig swing_traj_planner_config;
    swing_traj_planner_config.loadParams(nh);
    ElSpiderAirSimplePlanner planner(swing_traj_planner_config, fake_feedback, simulation);
    planner.run();
    return 0;
}