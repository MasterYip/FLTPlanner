#include "legged_traj_plan/whole_body_planner/StateSequencePlanner.h"

StateSequencePlanner::StateSequencePlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                           std::shared_ptr<GridMapInterface> gridmap_interface,
                                           std::shared_ptr<BaseRobotInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      use_cfg_space_(swing_traj_planner_config.useCfgSpace)
{
    if (swing_traj_planner_config.plannerID == 0)
    {
        if (use_cfg_space_)
        {
            swing_traj_planner_ = std::make_shared<FLTCfgPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_);
        }
        else
        {
            swing_traj_planner_ = std::make_shared<FLTPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_);
        }
    }
    else if (swing_traj_planner_config.plannerID == 1)
    {
        swing_traj_planner_ = std::make_shared<RRTPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_);
    }
    else if (swing_traj_planner_config.plannerID == 2)
    {
        swing_traj_planner_ = std::make_shared<RRTCfgPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_);
    }
    else
    {
        ROS_ERROR("Invalid planner ID");
    }
}

StateSequencePlanner::StateSequencePlanner(std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
                                           std::shared_ptr<GridMapInterface> gridmap_interface,
                                           std::shared_ptr<BaseRobotInterface> robot_interface)
    : swing_traj_planner_(swing_traj_planner),
      gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      use_cfg_space_(swing_traj_planner->getConfig().useCfgSpace)
{
}

bool StateSequencePlanner::enqueue_MCTsolution(hexapod_State state0, hexapod_State state1)
{
    if (enable_record_states_)
    {
        record_states_.push_back(state0);
    }
    state_trajs.emplace_back(MCTStateTransfer(state0, state1, swing_traj_planner_, use_cfg_space_));
    return true;
}

MCTStateTransfer StateSequencePlanner::dequeue_MCTsolution()
{
    MCTStateTransfer ret = state_trajs.front();
    assert(!state_trajs.empty());
    state_trajs.erase(state_trajs.begin());
    return ret;
}

MCTStateTransfer &StateSequencePlanner::get_state_traj(int index)
{
    return state_trajs.at(index);
}

int StateSequencePlanner::get_state_traj_length()
{
    return state_trajs.size();
}
