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
#include "legged_traj_plan/swing_leg_planner_rrt/SwingTrajOptRRT.h"

SwingTrajPlannerRRT::SwingTrajPlannerRRT(SwingTrajPlannerConfig config,
                                         std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                         std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                                swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                                                nullptr, config.enableBenchmark)
{
    // visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_rrt_vis");
    // if (config_.enableOptVis)
    //     swing_traj_opt_.setVisualizer(visualizer_);
}

std::shared_ptr<TrajectoryBase> SwingTrajPlannerRRT::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                                 uint index)
{
    double h_lift = config_.hLift;
    Eigen::MatrixXd knots(3, 3);
    knots.row(0) = p0;
    knots.row(1) = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);
    knots.row(2) = p1;
    return std::make_shared<UniBSpline>(knots);
}

bool SwingTrajPlannerRRT::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<UniBSpline> unib_traj = std::dynamic_pointer_cast<UniBSpline>(traj);
    swing_traj_opt_.optimize(*unib_traj, config_, 0.1);
}
