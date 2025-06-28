/**
 * @file unitree_a1_state_sequence_planner.cpp
 * @author GitHub Copilot
 * @brief Unitree A1 State Sequence Planner Node
 * @version 0.1
 * @date 2025-06-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "legged_traj_plan_examples/unitree_a1_planner/UnitreeA1StateSequencePlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_a1_state_sequence_planner");
    // Create planner
    UnitreeA1StateSequencePlanner planner;
    planner.run();
    return 0;
}