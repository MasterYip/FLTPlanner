/**
 * @file StateSequencePlanner.h
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

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/whole_body_planner/MCTStateTransfer.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/rrt_planner/RRTPlanner.h"

/* external project header files */
#include <geometry_msgs/Twist.h>

class StateSequencePlanner
{
private:
    std::vector<MCTStateTransfer> state_trajs;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;
    bool use_cfg_space_;

    bool enable_record_states_;
    std::vector<hexapod_State> record_states_;

public:
    [[deprecated]] StateSequencePlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                        std::shared_ptr<GridMapInterface> gridmap_interface,
                                        std::shared_ptr<ElSpiderAirInterface> robot_interface);

    StateSequencePlanner(std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
                         std::shared_ptr<GridMapInterface> gridmap_interface,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface);

    bool enqueue_MCTsolution(hexapod_State state0, hexapod_State state1);

    MCTStateTransfer dequeue_MCTsolution();

    MCTStateTransfer &get_state_traj(int index);

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

    int get_state_traj_length();

    void saveBenchmarkResults(void)
    {
        swing_traj_planner_->saveBenchmarkResults();
    }

    void enableRecordStates(bool enable)
    {
        enable_record_states_ = enable;
    }

    std::vector<hexapod_State> &getRecordStates()
    {
        return record_states_;
    }

    void visClear()
    {
        swing_traj_planner_->visClear();
    }
};
