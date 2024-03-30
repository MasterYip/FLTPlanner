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

std::shared_ptr<TrajectoryBase> SwingTrajPlanner::getDefaultTraj(Eigen::Vector3d &p0, Eigen::Vector3d &p1, double v_lift, double h_lift)
{
    // UniBSpline
    // Eigen::Vector3d pm = (p0 + p1) / 2;
    // pm(2) += h_lift;
    // Eigen::MatrixXd knots(3, 3);
    // knots << p0.transpose(), pm.transpose(), p1.transpose();
    // return std::make_shared<UniBSpline>(knots);

    // Minco
    minco::MINCO_S2NU minco;
    Eigen::Matrix<double, 3, 2> head_state;
    Eigen::Matrix<double, 3, 2> tail_state;
    Eigen::Matrix3Xd knots(3, 1);
    Eigen::VectorXd ts(2);
    head_state.col(0) = p0;
    head_state.col(1) = Eigen::Vector3d(0, 0, v_lift);
    tail_state.col(0) = p1;
    tail_state.col(1) = Eigen::Vector3d(0, 0, -v_lift);
    knots.col(0) = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);
    ts << 0.5, 0.5;
    minco.setConditions(head_state, tail_state, 2);
    minco.setParameters(knots, ts);
    return std::make_shared<MincoTrajectory>(minco);
}

// TODO:
bool SwingTrajPlanner::opt_traj(std::shared_ptr<TrajectoryBase> traj, int index)
{
    return true;
}
