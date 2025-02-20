/**
 * @file MCTStateTransfer.h
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
#include <vector>
/* external project header files */
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
/* internal project header files */
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"

using hexapod_State = legged_traj_plan::hexapod_State;
using PosList = std::vector<Eigen::Vector3d>;

class MCTStateTransfer
{
private:
    hexapod_State state0_;
    hexapod_State state1_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;
    PosList footpos_list0_;
    PosList footpos_list1_;
    std::shared_ptr<TrajectoryBase> swingtraj_[6];
    std::vector<bool> swingtraj_isopt_;
    std::vector<bool> swingtraj_isneeded_;
    bool use_cfg_space_ = false;

public:
    MCTStateTransfer(hexapod_State state0, hexapod_State state1,
                     std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner, bool use_cfg_space = false);
    pinocchio::SE3 eval_torso_traj(double t);
    PosList eval_foot_traj(double t, uint derivative = 0, bool auto_opt = true);
    PosList eval_cfg_traj(double t, uint derivative = 0, bool auto_opt = true);

    // PROBLEM: margin too small will leads to unstable gait switch in VMC controller?
    std::array<bool, 6> eval_support_state(double t, double lift_margin = 0.0, double touch_margin = 0.0);
    void opt_swing_traj(int index);
    void opt_swing_traj()
    {
        for (int i = 0; i < 6; ++i)
            opt_swing_traj(i);
    }
    std::vector<Eigen::Vector3d> generate_footholds(int index, int size=40, double interval=0.025);
    void reachable_check(int index);
    void reachable_check()
    {
        for (int i = 0; i < 6; ++i)
            reachable_check(i);
    }
    bool opt_check(int index);
};
