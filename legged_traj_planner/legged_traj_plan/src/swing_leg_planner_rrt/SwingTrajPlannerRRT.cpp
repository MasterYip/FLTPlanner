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

std::shared_ptr<UniBSpline> SwingTrajPlannerRRT::getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                                double h_lift)
{
    Eigen::MatrixXd knots(3, 3);
    knots.row(0) = p0;
    knots.row(1) = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);
    knots.row(2) = p1;
    return std::make_shared<UniBSpline>(knots);
}

// std::shared_ptr<UniBSpline> getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
//                                               const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
//                                               double h_lift = 0.1)
// {
//     std::vector<Point3D> cfg_poly_traj;
//     std::vector<Point3D> poly_path;
//     poly_path.emplace_back(p0);
//     poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
//     poly_path.emplace_back(p1);
//     // Convert to config space
//     Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, config_.trajTime);
//     double t = 0;
//     for (uint i = 0; i < poly_path.size() - 1; i++)
//     {
//         // Convert to base frame
//         Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
//         cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
//         t += t_vec(i);
//     }

//     // Get start and goal velocity in config space
//     // FIXME: the vel is in BASE frame, not in WORLD frame
//     Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
//     Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
//     Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
//     Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
//     start_vel = J_inv * start_vel;
//     J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
//     J_inv = J.transpose() * (J * J.transpose()).inverse();
//     goal_vel = J_inv * goal_vel;
// }

bool SwingTrajPlannerRRT::optTraj(std::shared_ptr<TrajectoryBase> &traj)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<MincoTrajectory> unib_traj = std::dynamic_pointer_cast<UniBSpline>(traj);
    swing_traj_opt_.optimize( *unib_traj, config_, 0.1);
}
