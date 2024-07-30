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
                                                                                                swing_traj_opt_(config, robot_interface_, gridmap_interface_,
                                                                                                                nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
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
    std::shared_ptr<UniBSpline> unib_traj = std::make_shared<UniBSpline>(knots);
    // if (config_.enableVis)
    // {
    //     std::vector<Eigen::Vector3d> traj_points;
    //     unib_traj->getTrajSamples<Eigen::Vector3d>(traj_points, 100);
    //     visualizer_->visCurve(traj_points);
    // }
    return unib_traj;
}

bool SwingTrajPlannerRRT::optTraj(std::shared_ptr<TrajectoryBase> &traj,
                                  const pinocchio::SE3 &pose0,
                                  const pinocchio::SE3 &pose1,
                                  int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<UniBSpline> unib_traj = std::dynamic_pointer_cast<UniBSpline>(traj);
    bool ret = swing_traj_opt_.optimize(*unib_traj, index);
    if (config_.enableVis && ret)
    {
        // Discrete
        std::vector<Eigen::Vector3d> rrt_poly_traj;
        Eigen::MatrixXd knots = unib_traj->get();
        for (int i = 0; i < knots.rows(); i++)
        {
            rrt_poly_traj.push_back(knots.row(i));
        }
        visualizer_->visCurve(rrt_poly_traj, ros_visualizer::VisStyle(0.3, 0.7, 0.3, 0.7, 0.01));
        // UniBSpline
        std::vector<Eigen::Vector3d> traj_points;
        unib_traj->getTrajSamples<Eigen::Vector3d>(traj_points, 100);
        visualizer_->visCurve(traj_points, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 1, 0.01));
    }
    return ret;
}

// SwingCfgTrajPlannerRRT

SwingCfgTrajPlannerRRT::SwingCfgTrajPlannerRRT(SwingTrajPlannerConfig config,
                                               std::shared_ptr<ElSpiderAirInterface> robot_interface,
                                               std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                                      swing_traj_opt_(config, robot_interface_, gridmap_interface_,
                                                                                                                      nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
}

std::shared_ptr<TrajectoryBase> SwingCfgTrajPlannerRRT::getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
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

    // if (config_.enableVis)
    // {
    //     std::vector<Eigen::Vector3d> traj_points;
    //     unib_traj->getTrajSamples<Eigen::Vector3d>(traj_points, 100);
    //     visualizer_->visCurve(traj_points);
    // }
    // return unib_traj;
}

// void SwingCfgTrajPlannerRRT::visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
//                                              std::vector<Point3D> cfg_poly_traj,
//                                              Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
//                                              int groupId)
// {
//     MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, trajTime);
//     std::vector<Point3D> cfg_path_opt;
//     std::vector<Point3D> path_opt;
//     double ts = 0.01;
//     double t = 0;
//     minco_traj.getTrajSamples(cfg_path_opt, ts);
//     for (auto pt : cfg_path_opt)
//     {
//         Point3D base_pt = robot_interface_->FK_foot(pt, index);
//         path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
//         t += ts;
//     }
//     visualizer_->setIdGroup(groupId);
//     visualizer_->visCurve(path_opt);
// }

bool SwingCfgTrajPlannerRRT::optTraj(std::shared_ptr<TrajectoryBase> &traj,
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
