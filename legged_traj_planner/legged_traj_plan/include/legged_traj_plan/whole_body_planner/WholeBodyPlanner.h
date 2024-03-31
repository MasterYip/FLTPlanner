/**
 * @file WholeBodyPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

#include "legged_traj_plan/utils/CircleQueue.h"
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/whole_body_planner/MCTStateTransfer.h"
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
class HITSpiderWholeBodyPlanner
{
private:
    std::vector<MCTStateTransfer> state_trajs;
    SwingTrajPlanner swing_traj_planner;
    // TODO: use shared_ptr
    BaseRobotInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;

public:
    // HITSpiderWholeBodyPlanner();

    HITSpiderWholeBodyPlanner(GridMapInterface &gridmap_interface, BaseRobotInterface &robot_interface);

    bool enqueue_MCTsolution(hexapod_State state0, hexapod_State state1);

    MCTStateTransfer dequeue_MCTsolution();

    MCTStateTransfer get_state_traj(int index);

    int get_state_traj_length();

    // std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> get_foot_traj(double t, int point_num, double delta);
};
