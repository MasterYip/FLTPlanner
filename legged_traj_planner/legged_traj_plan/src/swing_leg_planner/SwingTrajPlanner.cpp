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

#include <iostream>
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_search/poly_traj/poly_traj_search.hpp"
#include "legged_traj_search/traj_opt/minco_trajopt.hpp"

SwingTrajPlanner::SwingTrajPlanner(BaseRobotInterface &robot_interface,
                                   GridMapInterface &gridmap_interface) : robot_interface_(robot_interface),
                                                                          gridmap_interface_(gridmap_interface),
                                                                          visualizer_(nh_, "odom", "swing_traj_planner_vis")
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

std::shared_ptr<TrajectoryBase> SwingTrajPlanner::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                              Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                              double v_lift, uint index)
{
    Eigen::Matrix3Xd hull = robot_interface_.getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_.getMap(),
                                    gridmap_interface_.getGroundLayerName(),
                                    gridmap_interface_.getCeilingLayerName(), true, false);
    if (index == 0)
        visualizer_.delAll();
    visualizer_.visPolytope(corridor.getCorridor());
    visualizer_.visSphere(p0, 0.01);
    visualizer_.visSphere(p1, 0.01);

    if (!poly_traj_search.endpointValid(p0, p1))
    {
        std::cout << "Warning: poly_traj_search.endpointValid failed (leg " << index << ")" << std::endl;
        return getDefaultTraj(p0, p1, v_lift);
    }
    if (!poly_traj_search.reachable(p0, p1))
    {
        std::cout << "Warning: poly_traj_search.reachable failed" << std::endl;
        return getDefaultTraj(p0, p1, v_lift);
    }
    std::vector<Point3D> poly_path;
    if (!poly_traj_search.search(p0, p1, poly_path))
    {
        std::cout << "Warning: poly_traj_search.search failed" << std::endl;
        return getDefaultTraj(p0, p1, v_lift);
    }
    MincoTrajOpt minco_traj_opt(poly_path);
    return std::make_shared<MincoTrajectory>(minco_traj_opt.getTraj());
}

// TODO:
bool SwingTrajPlanner::opt_traj(std::shared_ptr<TrajectoryBase> traj, int index)
{
    return true;
}
