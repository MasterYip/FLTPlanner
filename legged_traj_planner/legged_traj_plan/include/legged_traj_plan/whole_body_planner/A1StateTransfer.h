/**
 * @file A1StateTransfer.h
 * @author GitHub Copilot
 * @brief A1 State Transfer for Unitree A1 Robot
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
#include <vector>
/* external project header files */
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
/* internal project header files */
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_plan/A1_State.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"

using A1_State = legged_traj_plan::A1_State;
using PosList = std::vector<Eigen::Vector3d>;

class A1StateTransfer
{
private:
    A1_State state0_;
    A1_State state1_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;
    PosList footpos_list0_;
    PosList footpos_list1_;
    std::shared_ptr<TrajectoryBase> swingtraj_[4]; // 4 legs for A1
    std::vector<bool> swingtraj_isopt_;
    std::vector<bool> swingtraj_isneeded_;
    bool use_cfg_space_ = false;

public:
    A1StateTransfer(A1_State state0, A1_State state1,
                    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner, bool use_cfg_space = false);
    pinocchio::SE3 eval_torso_traj(double t);
    PosList eval_foot_traj(double t, uint derivative = 0, bool auto_opt = true);
    PosList eval_cfg_traj(double t, uint derivative = 0, bool auto_opt = true);

    // Support state evaluation for 4 legs
    std::array<bool, 4> eval_support_state(double t, double lift_margin = 0.0, double touch_margin = 0.0);
    void opt_swing_traj(int index);
    void opt_swing_traj()
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
            opt_swing_traj(i);
    }
    std::vector<Eigen::Vector3d> generate_footholds(int index, int size = 30, double interval = 0.025);
    void reachable_check(int index, int size = 30, double interval = 0.025);
    void reachable_check()
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
            reachable_check(i);
    }
    bool opt_check(int index);
};