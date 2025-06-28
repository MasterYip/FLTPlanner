/**
 * @file A1StateSequencePlanner.cpp
 * @author GitHub Copilot
 * @brief A1 State Sequence Planner Implementation for Unitree A1 Robot
 * @version 0.1
 * @date 2025-06-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "legged_traj_plan/whole_body_planner/A1StateSequencePlanner.h"

A1StateSequencePlanner::A1StateSequencePlanner(std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
                                               std::shared_ptr<GridMapInterface> gridmap_interface,
                                               std::shared_ptr<BaseRobotInterface> robot_interface)
    : swing_traj_planner_(swing_traj_planner),
      gridmap_interface_(gridmap_interface),
      robot_interface_(robot_interface),
      use_cfg_space_(swing_traj_planner->getConfig().useCfgCommand),
      enable_record_states_(false)
{
}

bool A1StateSequencePlanner::enqueue_A1solution(A1_State state0, A1_State state1)
{
    state_trajs.emplace_back(A1StateTransfer(state0, state1, swing_traj_planner_, use_cfg_space_));

    if (enable_record_states_)
    {
        record_states_.push_back(state0);
        record_states_.push_back(state1);
    }

    return true;
}

A1StateTransfer A1StateSequencePlanner::dequeue_A1solution()
{
    if (state_trajs.empty())
        throw std::runtime_error("No state trajectories available to dequeue");

    A1StateTransfer front = state_trajs.front();
    state_trajs.erase(state_trajs.begin());
    return front;
}

A1StateTransfer &A1StateSequencePlanner::get_state_traj(int index)
{
    if (index >= state_trajs.size())
        throw std::out_of_range("Index out of range for state trajectories");

    return state_trajs[index];
}

int A1StateSequencePlanner::get_state_traj_length()
{
    return state_trajs.size();
}