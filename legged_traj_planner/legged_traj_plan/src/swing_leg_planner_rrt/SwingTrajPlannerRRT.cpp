/**
 * @file SwingTrajPlannerRRT.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-07-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include "legged_traj_plan/swing_leg_planner_rrt/SwingTrajPlannerRRT.h"


SwingTrajPlanner::SwingTrajPlanner(SwingTrajPlannerConfig config,
                                   std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                   std::shared_ptr<GridMapInterface> gridmap_interface) : robot_interface_(robot_interface),
                                                                                          gridmap_interface_(gridmap_interface),
                                                                                          swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                                          nullptr, config.enableBenchmark),
                                                                                          config_(config)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_rrt_vis");
    if (config_.enableOptVis)
        swing_traj_opt_.setVisualizer(visualizer_);
}

std::shared_ptr<MincoTrajectory> SwingTrajPlannerRRT::getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                                     double v_lift, double h_lift = 0.1);

std::shared_ptr<MincoTrajectory> SwingTrajPlannerRRT::getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                                        const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                                                                        double v_lift, double h_lift = 0.1);

std::shared_ptr<MincoTrajectory> SwingTrajPlannerRRT::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                  Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                  double v_lift, double h_lift, uint index);

bool SwingTrajPlannerRRT::getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                                         pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                         Eigen::Vector3d p0, Eigen::Vector3d p1,
                                         uint index);
std::shared_ptr<MincoTrajectory> SwingTrajPlannerRRT::getCfgInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                     Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                     double v_lift, uint index);

bool SwingTrajPlannerRRT::optCfgTraj(std::shared_ptr<TrajectoryBase> &traj,
                                     const pinocchio::SE3 &pose0,
                                     const pinocchio::SE3 &pose1,
                                     int index);

bool SwingTrajPlannerRRT::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index);

std::shared_ptr<ElSpiderAirInterface> SwingTrajPlannerRRT::getRobotInterface()
{
    return robot_interface_;
}

const SwingTrajPlannerConfig &SwingTrajPlannerRRT::getConfig() const
{
    return config_;
}

void SwingTrajPlannerRRT::visClear()
{
    visualizer_->delAll();
}

void SwingTrajPlannerRRT::saveBenchmarkResults();
