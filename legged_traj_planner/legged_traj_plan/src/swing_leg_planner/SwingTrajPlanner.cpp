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

#define ENABLE_VISUALIZER

pinocchio::SE3 poseLinearInterp(pinocchio::SE3 pose0, pinocchio::SE3 pose1, double t)
{
    pinocchio::Motion err = pinocchio::log6(pose0.actInv(pose1));
    pinocchio::SE3 interp = pose0.act(pinocchio::exp6(err * t));
    return interp;
}

Eigen::VectorXd getTrajTimeVec(const std::vector<Point3D> &path, double total_time)
{
    if (path.size() > 2)
    {
        double path_length = 0;
        Eigen::VectorXd ts(path.size() - 1);
        for (size_t i = 1; i < path.size() - 1; i++)
            path_length += (path[i] - path[i - 1]).norm();
        path_length += (path.back() - path[path.size() - 2]).norm();
        for (size_t i = 1; i < path.size(); i++)
            ts[i - 1] = (path[i] - path[i - 1]).norm() / path_length * total_time;
        return ts;
    }
    else
    {
        Eigen::VectorXd ts(1);
        ts << total_time;
        return ts;
    }
}

SwingTrajPlanner::SwingTrajPlanner(ElSpiderAirInterface &robot_interface,
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

bool SwingTrajPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                      const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                      const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                      uint index, bool verbose)
{
    poly_traj.clear();
    Eigen::Matrix3Xd hull = robot_interface_.getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_.getMap(),
                                    gridmap_interface_.getGroundLayerName(),
                                    gridmap_interface_.getCeilingLayerName(), true, false);

    if (!poly_traj_search.endpointValid(p0, p1))
    {
        if (verbose)
            std::cout << "Warning: poly_traj_search.endpointValid failed (leg " << index << ")" << std::endl;
        return false;
    }
    if (!poly_traj_search.reachable(p0, p1))
    {
        if (verbose)
            std::cout << "Warning: poly_traj_search.reachable failed" << std::endl;
        return false;
    }
    if (!poly_traj_search.search(p0, p1, poly_traj))
    {
        if (verbose)
            std::cout << "Warning: poly_traj_search.search failed" << std::endl;
        return false;
    }

#ifdef ENABLE_VISUALIZER
    if (verbose && index == 0)
    {
        visualizer_.delAll();
        // Polytope
        visualizer_.visPolytope(corridor.getCorridor());
        // Start Goal
        visualizer_.visSphere(p0, 0.02);
        visualizer_.visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search.getBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy;
            pos[2] = poly_traj_search.getBorderCheck().queryHeight(border.at(i));
            gridmap_interface_.getMap().getPosition(border.at(i), posxy);
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        border_pos.push_back(border_pos.front());
        visualizer_.visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        // Poly Path
        visualizer_.visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }
#endif

    return true;
}

std::shared_ptr<TrajectoryBase> SwingTrajPlanner::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                              Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                              double v_lift, uint index)
{
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultTraj(p0, p1, v_lift);
    }
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    MincoTrajOpt minco_traj_opt(poly_path, start_vel, goal_vel, 1.0);
#ifdef ENABLE_VISUALIZER
    // Minco
    std::vector<Point3D> poly_path_opt;
    minco_traj_opt.getTrajSamples(poly_path_opt);
    visualizer_.visCurve(poly_path_opt);
#endif

    return std::make_shared<MincoTrajectory>(minco_traj_opt.getTraj());
}

std::shared_ptr<TrajectoryBase> SwingTrajPlanner::getCfgInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                 double v_lift, uint index)
{
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultTraj(p0, p1, v_lift);
    }
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, 1.0);
    std::vector<Point3D> cfg_poly_path;
    double t = 0;
    for (uint i = 0; i < poly_path.size() - 1; i++)
    {
        // Convert to base frame
        Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
        cfg_poly_path.emplace_back(robot_interface_.IKFast_foot(base_pt, index));
        t += t_vec(i);
    }
    Point3D base_pt = point_SE3Act(pose1, poly_path.back());
    cfg_poly_path.emplace_back(robot_interface_.IKFast_foot(base_pt, index));

    // Get start and goal velocity in config space
    // FIXME: the vel is in BASE frame, not in WORLD frame
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    Eigen::Matrix3Xd J = robot_interface_.getJacobian(cfg_poly_path.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_.getJacobian(cfg_poly_path.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    MincoTrajOpt minco_traj_opt(cfg_poly_path, start_vel, goal_vel, 1.0);
#ifdef ENABLE_VISUALIZER
    // Minco
    std::vector<Point3D> cfg_path_opt;
    std::vector<Point3D> path_opt;
    double ts = 0.01;
    t = 0;
    minco_traj_opt.getTrajSamples(cfg_path_opt, ts);
    for (auto pt : cfg_path_opt)
    {
        Point3D base_pt = robot_interface_.FK_foot(pt, index);
        path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
        t += ts;
    }
    visualizer_.visCurve(path_opt);
#endif
    return std::make_shared<MincoTrajectory>(minco_traj_opt.getTraj());
}

// TODO:
bool SwingTrajPlanner::opt_traj(std::shared_ptr<TrajectoryBase> traj, int index)
{
    return true;
}
