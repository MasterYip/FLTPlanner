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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
class MCTSWholeBodyPlanner
{
private:
    std::vector<MCTStateTransfer> state_trajs;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlanner> swing_traj_planner_;
    bool use_cfg_space_;

public:
    MCTSWholeBodyPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                         std::shared_ptr<GridMapInterface> gridmap_interface,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface, bool use_cfg_space = true);

    bool enqueue_MCTsolution(hexapod_State state0, hexapod_State state1);

    MCTStateTransfer dequeue_MCTsolution();

    MCTStateTransfer get_state_traj(int index);

    int get_state_traj_length();

    // std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> get_foot_traj(double t, int point_num, double delta);
};
