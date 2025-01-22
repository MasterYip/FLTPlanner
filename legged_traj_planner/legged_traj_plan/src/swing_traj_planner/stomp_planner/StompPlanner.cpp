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

std::shared_ptr<TrajectoryBase> StompCfgPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                 uint index)
{
    double h_lift = config_.hLift;
    double v_lift = config_.vLift;
    std::vector<Point3D> poly_path;
    poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose0, p0), index));
    poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose1, p1), index));
    // knots.row(0) = p0;
    // knots.row(1) = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);
    // knots.row(2) = p1;

    return std::make_shared<MincoTrajectory>(poly_path, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), config_.trajTime);
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
    reachable.resize(footholds.size());
    for (int i = 0; i < footholds.size(); i++)
    {
        if (ifEndPointKinValid(pose0, pose1, p0, footholds.at(i), index))
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