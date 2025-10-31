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

#include "legged_traj_plan_examples/backward.hpp"
#include "legged_traj_plan_examples/hexapod201_planner/Hexapod201StateSequencePlanner.h"
namespace backward {

backward::SignalHandling sh;

} // namespace backward

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod201_state_sequence_planner");
    // Create planner
    Hexapod201StateSequencePlanner planner;
    planner.run();
    return 0;
}