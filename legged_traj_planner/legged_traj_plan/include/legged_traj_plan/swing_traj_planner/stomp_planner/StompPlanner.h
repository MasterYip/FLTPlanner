/**
 * @file STOMPPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "SwingTrajOptStomp.h"

class StompPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    std::shared_ptr<StompTask> swing_traj_opt_;

public:
    StompPlanner(SwingTrajPlannerConfig config,
                 std::shared_ptr<ElSpiderAirInterface> robot_interface,
                 std::shared_ptr<GridMapInterface> gridmap_interface);
    ~StompPlanner() = default;

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index);

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index);
};

class StompCfgPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    std::shared_ptr<CfgStompTask> swing_traj_opt_;

public:
    StompCfgPlanner(SwingTrajPlannerConfig config,
                    std::shared_ptr<ElSpiderAirInterface> robot_interface,
                    std::shared_ptr<GridMapInterface> gridmap_interface);
    ~StompCfgPlanner() = default;

    void visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
                         std::vector<Point3D> cfg_poly_traj,
                         Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
                         int groupId = 1);

    bool searchPolyTrajPITD(std::vector<Point3D> &poly_traj,
                            const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                            const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                            uint index, bool verbose = true);

    bool getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                        pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                        Eigen::Vector3d p0, Eigen::Vector3d p1,
                        uint index);

    std::shared_ptr<MincoTrajectory> getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                       const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index);

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index);

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index);

    bool reachableCheckHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                            Eigen::Vector3d p0, uint index,
                            std::vector<Eigen::Vector3d> &footholds,
                            std::vector<bool> &reachable) override;
};