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

#define ENABLE_VISUALIZER

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

SwingTrajPlanner::SwingTrajPlanner(SwingTrajPlannerConfig config,
                                   std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                   std::shared_ptr<GridMapInterface> gridmap_interface) : robot_interface_(robot_interface),
                                                                                          gridmap_interface_(gridmap_interface),
                                                                                          swing_traj_opt_(robot_interface_, gridmap_interface_),
                                                                                          config_(config)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
        swing_traj_opt_.setVisualizer(visualizer_);
}

std::shared_ptr<MincoTrajectory> SwingTrajPlanner::getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                                  double v_lift, double h_lift)
{
    // UniBSpline
    // Eigen::Vector3d pm = (p0 + p1) / 2;
    // pm(2) += h_lift;
    // Eigen::MatrixXd knots(3, 3);
    // knots << p0.transpose(), pm.transpose(), p1.transpose();
    // return std::make_shared<UniBSpline>(knots);

    // Minco
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
    poly_path.emplace_back(p1);
    return std::make_shared<MincoTrajectory>(poly_path, start_vel, goal_vel, 1.0);
}

std::shared_ptr<MincoTrajectory> SwingTrajPlanner::getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                                     const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                                                                     double v_lift, double h_lift)
{
    std::vector<Point3D> cfg_poly_traj;
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
    poly_path.emplace_back(p1);
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, 1.0);
    double t = 0;
    for (uint i = 0; i < poly_path.size() - 1; i++)
    {
        // Convert to base frame
        Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
        cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
        t += t_vec(i);
    }

    // Get start and goal velocity in config space
    // FIXME: the vel is in BASE frame, not in WORLD frame
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    return std::make_shared<MincoTrajectory>(cfg_poly_traj, start_vel, goal_vel, 1.0);
}

bool SwingTrajPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                      const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                      const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                      uint index, bool verbose)
{
    poly_traj.clear();
    Eigen::Matrix3Xd hull = robot_interface_->getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_->getMap(),
                                    gridmap_interface_->getGroundLayerName(),
                                    gridmap_interface_->getCeilingLayerName(), true, false);

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
        visualizer_->delAll();
        visualizer_->setIdGroup(1);
        // Polytope
        visualizer_->visPolytope(corridor.getCorridor());
        // Start Goal
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search.getBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy;
            pos[2] = poly_traj_search.getBorderCheck().queryHeight(border.at(i));
            gridmap_interface_->getMap().getPosition(border.at(i), posxy);
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        border_pos.push_back(border_pos.front());
        visualizer_->visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        // Poly Path
        visualizer_->visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }
#endif

    return true;
}

std::shared_ptr<MincoTrajectory> SwingTrajPlanner::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                               Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                               double v_lift, double h_lift, uint index)
{
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultTraj(p0, p1, v_lift, h_lift);
    }
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    MincoTrajectory minco_traj(poly_path, start_vel, goal_vel, 1.0);
#ifdef ENABLE_VISUALIZER
    // Minco
    visualizer_->setIdGroup(1);
    std::vector<Point3D> poly_path_opt;
    minco_traj.getTrajSamples(poly_path_opt);
    visualizer_->visCurve(poly_path_opt);
#endif

    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool SwingTrajPlanner::getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                                      pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                      Eigen::Vector3d p0, Eigen::Vector3d p1,
                                      uint index)
{
    cfg_poly_traj.clear();
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
        return false;
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, 1.0);
    double t = 0;
    for (uint i = 0; i < poly_path.size() - 1; i++)
    {
        // Convert to base frame
        Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
        cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
        t += t_vec(i);
    }
    Point3D base_pt = point_SE3Act(pose1, poly_path.back());
    cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
    return true;
}

std::shared_ptr<MincoTrajectory> SwingTrajPlanner::getCfgInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                  Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                  double v_lift, uint index)
{
    std::vector<Point3D> cfg_poly_path;
    if (!getCfgPolyTraj(cfg_poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultCfgTraj(pose0, pose1, p0, p1, v_lift, index);
    }

    // Get start and goal velocity in config space
    // FIXME: the vel is in BASE frame, not in WORLD frame
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_path.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_path.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    MincoTrajectory minco_traj(cfg_poly_path, start_vel, goal_vel, 1.0);
#ifdef ENABLE_VISUALIZER
    // Minco
    std::vector<Point3D> cfg_path_opt;
    std::vector<Point3D> path_opt;
    double ts = 0.01;
    double t = 0;
    minco_traj.getTrajSamples(cfg_path_opt, ts);
    for (auto pt : cfg_path_opt)
    {
        Point3D base_pt = robot_interface_->FK_foot(pt, index);
        path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
        t += ts;
    }
    visualizer_->setIdGroup(1);
    visualizer_->visCurve(path_opt);
#endif
    return std::make_shared<MincoTrajectory>(minco_traj);
}

// TODO:
bool SwingTrajPlanner::optCfgTraj(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index)
{
    // FIXME: Temporarily cast to MincoTrajectory
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    // FIXME: start_vel & goal set to zero (test)
    start_vel = Eigen::Vector3d::Zero();
    goal_vel = Eigen::Vector3d::Zero();
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];
    std::cout << "Index: " << index << ", Poly path size: " << poly_path.size() << std::endl;

    swing_traj_opt_.setup(pose0, pose1, index, poly_path_mat, start_vel, goal_vel,
                          config_, true);
    bool ret = swing_traj_opt_.optimize(minco_traj->getTraj(), config_.relCostTol);

#ifdef ENABLE_VISUALIZER
    if (ret)
    {
        // Minco
        std::vector<Point3D> cfg_path_opt;
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        minco_traj->getTrajSamples(cfg_path_opt, ts);

        for (auto pt : cfg_path_opt)
        {
            Point3D base_pt = robot_interface_->FK_foot(pt, index);
            path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
            t += ts;
        }
        visualizer_->setIdGroup(1);
        visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
    }
#endif
    return true;
}

bool SwingTrajPlanner::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                               const pinocchio::SE3 &pose0,
                               const pinocchio::SE3 &pose1,
                               int index)
{
    // FIXME: Temporarily cast to MincoTrajectory
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];
    std::cout << "Index: " << index << ", Poly path size: " << poly_path.size() << std::endl;

    swing_traj_opt_.setup(pose0, pose1, index, poly_path_mat, start_vel, goal_vel,
                          config_, false);
    bool ret = swing_traj_opt_.optimize(minco_traj->getTraj(), config_.relCostTol);

#ifdef ENABLE_VISUALIZER
    if (ret)
    {
        // Minco
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        minco_traj->getTrajSamples(path_opt, ts);
        visualizer_->setIdGroup(1);
        visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
    }
#endif
    return true;
}