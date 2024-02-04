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
#include "fast_legged_planner/utils/Geometry.h"
#include "fast_legged_planner/hexapod_State.h"
#include "fast_legged_planner/swing_leg_planner/SwingTrajPlanner.h"

using hexapod_State = fast_legged_planner::hexapod_State;
using PosList = std::vector<Eigen::Vector3d>;

class MCTStateTransfer
{
private:
    hexapod_State state0;
    hexapod_State state1;
    SwingTrajPlanner swing_traj_planner;
    PosList footpos_list0;
    PosList footpos_list1;
    std::vector<UniBSpline> swingtraj;
    std::vector<bool> swingtraj_isopt;
    std::vector<bool> swingtraj_isneeded;

public:
    MCTStateTransfer(hexapod_State state0, hexapod_State state1, SwingTrajPlanner swing_traj_planner);
    pinocchio::SE3 eval_torso_traj(double t);
    PosList eval_foot_traj(double t, bool auto_opt = true);
    void opt_swing_traj(int index);
    bool opt_check(int index);
};
