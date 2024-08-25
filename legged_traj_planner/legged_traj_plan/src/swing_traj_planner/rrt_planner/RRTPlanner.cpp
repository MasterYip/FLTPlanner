/**
 * @file RRTPlanner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-07-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include "legged_traj_plan/swing_traj_planner/rrt_planner/RRTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/rrt_planner/SwingTrajOptRRT.h"

RRTPlanner::RRTPlanner(SwingTrajPlannerConfig config,
                       std::shared_ptr<ElSpiderAirInterface> robot_interface,
                       std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                              swing_traj_opt_(config, robot_interface_, gridmap_interface_,
                                                                                              nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
}

std::shared_ptr<TrajectoryBase> RRTPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                            Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                            uint index)
{
    double h_lift = config_.hLift;
    double v_lift = config_.vLift;
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
    poly_path.emplace_back(p1);
    return std::make_shared<MincoTrajectory>(poly_path, start_vel, goal_vel, config_.trajTime);
}

bool RRTPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                             const pinocchio::SE3 &pose0,
                             const pinocchio::SE3 &pose1,
                             int index)
{
    if (!config_.enableOptimizer)
        return true;
    bool ret = swing_traj_opt_.optimize(traj, index);
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    if (config_.enableVis && ret)
    {
        // Discrete
        std::vector<Eigen::Vector3d> rrt_poly_traj = minco_traj->getPolyPath();
        visualizer_->visCurve(rrt_poly_traj, ros_visualizer::VisStyle(0.3, 0.7, 0.3, 0.7, 0.01));
        // Minco
        std::vector<Eigen::Vector3d> traj_points;
        minco_traj->getTrajSamples(traj_points);
        visualizer_->visCurve(traj_points, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 1, 0.01));
    }
    return ret;
}

// RRTCfgPlanner

RRTCfgPlanner::RRTCfgPlanner(SwingTrajPlannerConfig config,
                             std::shared_ptr<ElSpiderAirInterface> robot_interface,
                             std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                    swing_traj_opt_(config, robot_interface_, gridmap_interface_,
                                                                                                    nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
}

std::shared_ptr<TrajectoryBase> RRTCfgPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                               Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                               uint index)
{
    double h_lift = config_.hLift;
    double v_lift = config_.vLift;
    std::vector<Point3D> poly_path;
    poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose0, p0), index));
    poly_path.push_back(robot_interface_->IKFast_foot(point_SE3Act(pose1, p1), index));

    return std::make_shared<MincoTrajectory>(poly_path, Eigen::Vector3d(0, 0, 0),
                                             Eigen::Vector3d(0, 0, 0), config_.trajTime);
}

bool RRTCfgPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                                const pinocchio::SE3 &pose0,
                                const pinocchio::SE3 &pose1,
                                int index)
{
    if (!config_.enableOptimizer)
        return true;
    bool ret = swing_traj_opt_.optimize(traj, pose0, pose1, index);
    if (config_.enableVis && ret)
    {
        // Discrete
        // std::vector<Eigen::Vector3d> rrt_poly_traj;
        // Eigen::MatrixXd knots = minco_traj->get();
        // for (int i = 0; i < knots.rows(); i++)
        // {
        //     rrt_poly_traj.push_back(knots.row(i));
        // }
        // visualizer_->visCurve(rrt_poly_traj, ros_visualizer::VisStyle(0.3, 0.7, 0.3, 0.7, 0.01));

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
        visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
    }
    return ret;
}
