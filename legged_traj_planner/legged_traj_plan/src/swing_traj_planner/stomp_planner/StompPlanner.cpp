/**
 * @file StompPlanner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/swing_traj_planner/stomp_planner/StompPlanner.h"
#include <stomp/stomp.h>
// Poly traj search for traj init
#include "legged_traj_plan/swing_traj_planner/flt_planner/LeggedBorderCheck.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_search/poly_traj/poly_traj_search.hpp"

StompPlanner::StompPlanner(SwingTrajPlannerConfig config,
                           std::shared_ptr<ElSpiderAirInterface> robot_interface,
                           std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                  swing_traj_opt_(std::make_shared<StompTask>(config, gridmap_interface_,
                                                                                                                              nullptr))
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
    {
        swing_traj_opt_->setupVis(visualizer_);
    }
}

std::shared_ptr<TrajectoryBase> StompPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                              Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                              uint index)
{
    double h_lift = config_.hLift;
    double v_lift = config_.vLift;
    std::vector<Point3D> poly_path{p0, (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift), p1};

    return std::make_shared<MincoTrajectory>(poly_path, Eigen::Vector3d(0, 0, v_lift), Eigen::Vector3d(0, 0, -v_lift), config_.trajTime);
}

bool StompPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                               const pinocchio::SE3 &pose0,
                               const pinocchio::SE3 &pose1,
                               int index)
{
    bool ret = true;
    if (!config_.enableOptimizer)
        return true;

    Eigen::Vector3d p0 = traj->evaluate(0, 0, true);
    Eigen::Vector3d p1 = traj->evaluate(1, 0, true);

    if (config_.enableVis)
    {
        visualizer_->setIdGroup(1);
        visualizer_->visSphere(p0);
        visualizer_->visSphere(p1);
    }

    swing_traj_opt_->setup(p0, p1);
    stomp::StompConfiguration c;
    c.num_timesteps = config_.stompNumTimesteps;
    c.num_iterations = config_.stompNumIters;
    c.num_dimensions = 3;
    c.delta_t = config_.trajTime / (config_.stompNumTimesteps - 1);
    c.control_cost_weight = config_.stompCtrlCostWeight;
    c.exponentiated_cost_sensitivity = config_.stompExpCostSensitivity;
    c.initialization_method = stomp::TrajectoryInitializations::MININUM_CONTROL_COST;
    c.num_iterations_after_valid = config_.stompNumItersAfterValid;
    c.num_rollouts = config_.stompNumRollouts;
    c.max_rollouts = config_.stompMaxRollouts;
    stomp::Stomp stomp(c, swing_traj_opt_);

    Eigen::MatrixXd opt_traj;
    if (stomp.solve(p0, p1, opt_traj))
        ret = true;
    else
    {
        std::cout << "A valid solution was not found" << std::endl;
        ret = false;
    }
    std::vector<Point3D> path_opt;
    for (int i = 0; i < opt_traj.cols(); i++)
    {
        path_opt.emplace_back(opt_traj.col(i));
    }

    traj = std::make_shared<MincoTrajectory>(path_opt, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), config_.trajTime);

    if (config_.enableVis)
    {
        // MincoTrajectory
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        std::dynamic_pointer_cast<MincoTrajectory>(traj)->getTrajSamples(path_opt, ts, true);
        visualizer_->setIdGroup(1);
        if (ret)
            visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
        else
            visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
    }
    return ret;
}

// StompCfgPlanner

StompCfgPlanner::StompCfgPlanner(SwingTrajPlannerConfig config,
                                 std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                 std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                        swing_traj_opt_(std::make_shared<CfgStompTask>(config, robot_interface_, gridmap_interface_,
                                                                                                                                       nullptr))
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
    {
        swing_traj_opt_->setupVis(visualizer_);
    }
}


void StompCfgPlanner::visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
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


std::shared_ptr<MincoTrajectory> StompCfgPlanner::getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                                  const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index)
{
    std::vector<Point3D> cfg_poly_traj;
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, config_.hLift));
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
    Point3D base_pt = point_SE3Act(pose1, poly_path.back());
    cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));

    // Get start and goal velocity in config space
    // NOTE: the vel is in BASE frame, not in WORLD frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d start_vel = vec_SE3Act(pose0, normal * config_.vLift); // Base frame
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d goal_vel = vec_SE3Act(pose1, -normal * config_.vLift); // Base frame
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;

    if (config_.enableVis)
    {
        visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    }

    return std::make_shared<MincoTrajectory>(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
}

// Interface to legged_traj_search
bool StompCfgPlanner::searchPolyTrajPITD(std::vector<Point3D> &poly_traj,
                                         const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                         const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                         uint index, bool verbose)
{
    poly_traj.clear();
    // gridmap_interface_->lockMapUpdate();
    std::unique_ptr<PolyTrajSearch> poly_traj_search;

    // Use LeggedBorderCheck
    LeggedBorderCheckConfig config;
    config.ground_layer = gridmap_interface_->getGroundLayerName();
    config.ceiling_layer = gridmap_interface_->getCeilingLayerName();
    config.enable_ground = true;
    config.enable_ceiling = false;
    config.collBallRad1 = config_.CollBall1Rad;
    config.collBallRad2 = config_.collBallCheckRad2;
    config.collBallRad3 = config_.collBallCheckRad3;
    auto border_check = std::make_shared<LeggedBorderCheck>(robot_interface_, gridmap_interface_,
                                                            pose0, pose1, p0, p1, index, config);
    PolyTrajSearchConfig cfg;
    cfg.enable_benchmark = false;
    poly_traj_search = std::make_unique<PolyTrajSearch>(border_check, gridmap_interface_->getMap(), cfg);

    bool ret_endpoint = poly_traj_search->endpointValid(p0, p1);
    bool ret_reachable = poly_traj_search->reachable(p0, p1);
    bool ret_search = poly_traj_search->search(p0, p1, poly_traj);
    // gridmap_interface_->unlockMapUpdate();

    if (verbose)
    {
        if (!ret_endpoint)
            std::cout << "Warning: poly_traj_search->endpointValid failed (leg " << index << ")" << std::endl;
        if (!ret_reachable)
            std::cout << "Warning: poly_traj_search->reachable failed" << std::endl;
        if (!ret_search)
            std::cout << "Warning: poly_traj_search->search failed" << std::endl;
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
    }

    if (config_.enableVis)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        // visualizer_->visPolytope(corridor.getCorridor());
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search->getFullResBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search->getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search->getBorderCheck()->queryHeight(border.at(i));
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        if (border_pos.size() > 0)
        {
            border_pos.push_back(border_pos.front());
            visualizer_->visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        }
        // // Vis Graph
        // VisibilityGraph vis_graph = poly_traj_search->getVisGraph();
        // CorridorBorderCheck border_check = poly_traj_search->getBorderCheck();
        // std::vector<Point3D> mesh;
        // uint size = vis_graph.size();
        // Point3D pos1, pos2;
        // for (uint i = 0; i < 1; i++)
        // {
        //     for (uint j = 0; j < size; j++)
        //     {
        //         if (i != j && vis_graph.isVisibile(i, j))
        //         {
        //             // pos1.head(2) = getPos(vis_graph.getPt(i));
        //             // pos2.head(2) = getPos(vis_graph.getPt(j));
        //             pos1.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(i));
        //             pos2.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(j));
        //             pos1[2] = border_check.queryHeight(vis_graph.getPt(i));
        //             pos2[2] = border_check.queryHeight(vis_graph.getPt(j));
        //             mesh.push_back(pos1);
        //             mesh.push_back(pos2);
        //         }
        //     }
        // }
        // visualizer_->visMesh(mesh, ros_visualizer::VisStyle(0.3, 0.3, 0.9, 0.6, 0.005));
        // Poly Path
        visualizer_->visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }

    return ret_endpoint && ret_reachable && ret_search;
}

bool StompCfgPlanner::getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                                     pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                     Eigen::Vector3d p0, Eigen::Vector3d p1,
                                     uint index)
{
    cfg_poly_traj.clear();
    std::vector<Point3D> poly_path;
    if (!searchPolyTrajPITD(poly_path, pose0, pose1, p0, p1, index))
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

std::shared_ptr<TrajectoryBase> StompCfgPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                 uint index)
{
    // double h_lift = config_.hLift;
    // double v_lift = config_.vLift;
    // std::vector<Point3D> poly_path;
    // poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose0, p0), index));
    // poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose1, p1), index));
    // // knots.row(0) = p0;
    // // knots.row(1) = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);
    // // knots.row(2) = p1;

    // return std::make_shared<MincoTrajectory>(poly_path, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), config_.trajTime);

    std::vector<Point3D> cfg_poly_traj;
    if (!config_.enablePolyPathSearch || !getCfgPolyTraj(cfg_poly_traj, pose0, pose1, p0, p1, index))
    {
        return getDefaultCfgTraj(pose0, pose1, p0, p1, index);
    }

    // Get start and goal velocity in config space
    // NOTE: the vel is in BASE frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d start_vel = vec_SE3Act(pose0, normal * config_.vLift); // Base frame
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d goal_vel = vec_SE3Act(pose1, -normal * config_.vLift); // Base frame
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    // start_vel[2] += 0.05;
    // goal_vel[2] -= 0.05;
    MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    if (config_.enableVis)
    {
        visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    }
    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool StompCfgPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index)
{
    bool ret = true;
    if (!config_.enableOptimizer)
        return true;

    Eigen::Vector3d p0cfg = traj->evaluate(0, 0, true);
    Eigen::Vector3d p1cfg = traj->evaluate(1, 0, true);
    Eigen::Vector3d p0 = point_SE3Act(pose0.inverse(), robot_interface_->FK_foot(p0cfg, index));
    Eigen::Vector3d p1 = point_SE3Act(pose1.inverse(), robot_interface_->FK_foot(p1cfg, index));

    if (config_.enableVis)
    {
        visualizer_->setIdGroup(1);
        visualizer_->visSphere(p0);
        visualizer_->visSphere(p1);
    }

    swing_traj_opt_->setup(pose0, pose1, p0, p1, index);
    stomp::StompConfiguration c;
    c.num_timesteps = config_.stompNumTimesteps;
    c.num_iterations = config_.stompNumIters;
    c.num_dimensions = 3;
    c.delta_t = config_.trajTime / (config_.stompNumTimesteps - 1);
    c.control_cost_weight = config_.stompCtrlCostWeight;
    c.exponentiated_cost_sensitivity = config_.stompExpCostSensitivity;
    c.initialization_method = stomp::TrajectoryInitializations::MININUM_CONTROL_COST;
    c.num_iterations_after_valid = config_.stompNumItersAfterValid;
    c.num_rollouts = config_.stompNumRollouts;
    c.max_rollouts = config_.stompMaxRollouts;
    stomp::Stomp stomp(c, swing_traj_opt_);

    Eigen::MatrixXd opt_traj;
    if (stomp.solve(p0cfg, p1cfg, opt_traj))
        ret = true;
    else
    {
        std::cout << "A valid solution was not found" << std::endl;
        ret = false;
    }
    std::vector<Point3D> cfg_path_opt;
    for (int i = 0; i < opt_traj.cols(); i++)
    {
        cfg_path_opt.emplace_back(opt_traj.col(i));
    }

    traj = std::make_shared<MincoTrajectory>(cfg_path_opt, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), config_.trajTime);

    if (config_.enableVis)
    {
        // MincoTrajectory
        std::vector<Point3D> cfg_path_opt;
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        std::dynamic_pointer_cast<MincoTrajectory>(traj)->getTrajSamples(cfg_path_opt, ts, true);
        for (auto pt : cfg_path_opt)
        {
            Point3D base_pt = robot_interface_->FK_foot(pt, index);
            path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
            t += ts;
        }
        visualizer_->setIdGroup(1);
        if (ret)
            visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
        else
            visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
    }
    return ret;
}

bool StompCfgPlanner::reachableCheckHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                         Eigen::Vector3d p0, uint index,
                                         std::vector<Eigen::Vector3d> &footholds,
                                         std::vector<bool> &reachable)
{
    for (int i = 0; i < footholds.size(); i++)
    {
        if (ifKinValid(pose1, footholds.at(i), index))
        {
            auto traj = getInitTrajHook(pose0, pose1, p0, footholds.at(i), index);
            if (optTrajHook(traj, pose0, pose1, index))
                reachable[i] = true;
            else
                reachable[i] = false;
        }
        else
            reachable[i] = false;
    }
    return true;
}