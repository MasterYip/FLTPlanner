/**
 * @file A1StateSequencePlanner.h
 * @author GitHub Copilot
 * @brief A1 State Sequence Planner for Unitree A1 Robot
 * @version 0.1
 * @date 2025-06-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/whole_body_planner/A1StateTransfer.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/rrt_planner/RRTPlanner.h"

/* external project header files */
#include <geometry_msgs/Twist.h>

class A1StateSequencePlanner
{
private:
    std::vector<A1StateTransfer> state_trajs;
    std::shared_ptr<BaseRobotInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;
    bool use_cfg_space_;

    bool enable_record_states_;
    std::vector<A1_State> record_states_;

public:
    A1StateSequencePlanner(std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
                           std::shared_ptr<GridMapInterface> gridmap_interface,
                           std::shared_ptr<BaseRobotInterface> robot_interface);

    bool enqueue_A1solution(A1_State state0, A1_State state1);

    A1StateTransfer dequeue_A1solution();

    A1StateTransfer &get_state_traj(int index);

    void optSwingTraj(int index)
    {
        state_trajs[index].opt_swing_traj();
    };

    void optSwingTraj()
    {
        for (int i = 0; i < state_trajs.size(); ++i)
        {
            state_trajs[i].opt_swing_traj();
        }
    };

    void reachableCheck(int leg_index, int index = 0)
    {
        state_trajs[index].reachable_check(leg_index);
    };

    void reachableCheck()
    {
        for (int i = 0; i < state_trajs.size(); ++i)
        {
            state_trajs[i].reachable_check();
        }
    };

    int get_state_traj_length();

    void saveBenchmarkResults(void)
    {
        swing_traj_planner_->saveBenchmarkResults();
    }

    void enableRecordStates(bool enable)
    {
        enable_record_states_ = enable;
    }

    std::vector<A1_State> &getRecordStates()
    {
        return record_states_;
    }

    void visClear()
    {
        swing_traj_planner_->visClear();
    }
};