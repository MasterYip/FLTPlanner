/**
 * @file SwingTrajPlanner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "fast_legged_planner/swing_leg_planner/SwingTrajPlanner.h"
#include <iostream>

SwingTrajPlanner::SwingTrajPlanner()
{
}

SwingTrajPlanner::~SwingTrajPlanner()
{
}

UniBSpline SwingTrajPlanner::get_default_traj(Eigen::Vector3d &p0, Eigen::Vector3d &p1, double v_lift, double h_lift)
{
    Eigen::Vector3d pm = (p0 + p1) / 2;
    pm(2) += h_lift;
    Eigen::MatrixXd knots(3, 3);
    knots << p0.transpose(), pm.transpose(), p1.transpose();
    return UniBSpline(knots);
}

// TODO:
bool SwingTrajPlanner::opt_traj(std::shared_ptr<TrajectoryBase> traj, int index)
{
    return true;
}
