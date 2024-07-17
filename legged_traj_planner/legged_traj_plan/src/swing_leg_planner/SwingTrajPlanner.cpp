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

////////////////////
// SwingTrajPlanner

SwingTrajPlanner::SwingTrajPlanner(SwingTrajPlannerConfig config,
                                   std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                   std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                          swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                                          nullptr, config.enableBenchmark)
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
    return std::make_shared<MincoTrajectory>(poly_path, start_vel, goal_vel, config_.trajTime);
}

bool SwingTrajPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                      const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                      const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                      uint index, bool verbose)
{
    poly_traj.clear();
    gridmap_interface_->lockMapUpdate();
    Eigen::Matrix3Xd hull = robot_interface_->getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_->getMap(),
                                    gridmap_interface_->getGroundLayerName(),
                                    gridmap_interface_->getCeilingLayerName(), true, false);
    if (verbose && index == 0 || 1)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        visualizer_->visPolytope(corridor.getCorridor());
    }

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
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
        if (verbose)
            std::cout << "Warning: poly_traj_search.search failed" << std::endl;
        return false;
    }
    gridmap_interface_->unlockMapUpdate();

#ifdef ENABLE_VISUALIZER
    if (verbose && index == 0 || 1)
    {
        visualizer_->setIdGroup(1);
        // Polytope
        // visualizer_->visPolytope(corridor.getCorridor());
        // Start Goal
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search.getBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search.getBorderCheck().getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search.getBorderCheck().queryHeight(border.at(i));
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

std::shared_ptr<TrajectoryBase> SwingTrajPlanner::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                              Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                              uint index)
{
    double v_lift = config_.vLift;
    double h_lift = config_.hLift;
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultTraj(p0, p1, v_lift, h_lift);
    }
    // Start & End Vel in World Frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    Eigen::Vector3d start_vel = normal * v_lift;
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    Eigen::Vector3d goal_vel = -normal * v_lift;
    MincoTrajectory minco_traj(poly_path, start_vel, goal_vel, config_.trajTime);
#ifdef ENABLE_VISUALIZER
    // Minco
    visualizer_->setIdGroup(1);
    std::vector<Point3D> poly_path_opt;
    minco_traj.getTrajSamples(poly_path_opt);
    visualizer_->visCurve(poly_path_opt);
#endif

    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool SwingTrajPlanner::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                               const pinocchio::SE3 &pose0,
                               const pinocchio::SE3 &pose1,
                               int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];

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

////////////////////
// SwingCfgTrajPlanner

SwingCfgTrajPlanner::SwingCfgTrajPlanner(SwingTrajPlannerConfig config,
                                         std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                         std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                                swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                                                nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
        swing_traj_opt_.setVisualizer(visualizer_);
}

void SwingCfgTrajPlanner::visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
                                          std::vector<Point3D> cfg_poly_traj,
                                          Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
                                          int groupId)
{
    MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, trajTime);
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
    visualizer_->setIdGroup(groupId);
    visualizer_->visCurve(path_opt);
}

std::shared_ptr<MincoTrajectory> SwingCfgTrajPlanner::getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                                        const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                                                                        double v_lift, double h_lift)
{
    std::vector<Point3D> cfg_poly_traj;
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
    poly_path.emplace_back(p1);
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, config_.trajTime);
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

#ifdef ENABLE_VISUALIZER
    visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
#endif

    return std::make_shared<MincoTrajectory>(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
}

bool SwingCfgTrajPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                         const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                         const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                         uint index, bool verbose)
{
    poly_traj.clear();
    gridmap_interface_->lockMapUpdate();
    Eigen::Matrix3Xd hull = robot_interface_->getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_->getMap(),
                                    gridmap_interface_->getGroundLayerName(),
                                    gridmap_interface_->getCeilingLayerName(), true, false);
    if (verbose && index == 0 || 1)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        visualizer_->visPolytope(corridor.getCorridor());
    }

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
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
        if (verbose)
            std::cout << "Warning: poly_traj_search.search failed" << std::endl;
        return false;
    }
    gridmap_interface_->unlockMapUpdate();

#ifdef ENABLE_VISUALIZER
    if (verbose && index == 0 || 1)
    {
        visualizer_->setIdGroup(1);
        // Polytope
        // visualizer_->visPolytope(corridor.getCorridor());
        // Start Goal
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search.getBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search.getBorderCheck().getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search.getBorderCheck().queryHeight(border.at(i));
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

bool SwingCfgTrajPlanner::getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                                         pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                         Eigen::Vector3d p0, Eigen::Vector3d p1,
                                         uint index)
{
    cfg_poly_traj.clear();
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
        return false;
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, config_.trajTime);
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

/**
 * @brief
 *
 * @param pose0
 * @param pose1
 * @param p0 In World frame
 * @param p1 In World frame
 * @param v_lift
 * @param index
 * @return std::shared_ptr<MincoTrajectory>
 */
std::shared_ptr<TrajectoryBase> SwingCfgTrajPlanner::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                 uint index)
{
    double v_lift = config_.vLift;
    std::vector<Point3D> cfg_poly_traj;
    if (!getCfgPolyTraj(cfg_poly_traj, pose0, pose1, p0, p1, index))
    {
        return getDefaultCfgTraj(pose0, pose1, p0, p1, index, v_lift);
    }

    // Get start and goal velocity in config space
    // NOTE: the vel is in BASE frame
    // FIXME: the vel is ill-directed when tested on hardware with vmc
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    Eigen::Vector3d start_vel = vec_SE3Act(pose0, normal * v_lift); // Base frame
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    Eigen::Vector3d goal_vel = vec_SE3Act(pose1, -normal * v_lift); // Base frame
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
#ifdef ENABLE_VISUALIZER
    visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
#endif
    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool SwingCfgTrajPlanner::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];

    swing_traj_opt_.setup(pose0, pose1, index, poly_path_mat, start_vel, goal_vel,
                          config_, true, false);
    bool ret = swing_traj_opt_.optimize(minco_traj->getTraj(), config_.relCostTol);

    if (config_.enableBenchmark)
    {
        benchmark_results_.emplace_back(swing_traj_opt_.getBenchmarkResult());
    }

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
