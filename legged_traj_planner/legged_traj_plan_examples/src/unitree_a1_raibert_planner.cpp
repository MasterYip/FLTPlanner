/**
 * @file unitree_a1_raibert_planner.cpp
 * @author GitHub Copilot
 * @brief Unitree A1 Raibert Planner - Quadruped implementation
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "legged_traj_plan_examples/unitree_a1_planner/UnitreeA1RaibertPlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_a1_raibert_planner");
    // Create planner
    UnitreeA1RaibertPlanner planner;
    planner.run();
    return 0;
}