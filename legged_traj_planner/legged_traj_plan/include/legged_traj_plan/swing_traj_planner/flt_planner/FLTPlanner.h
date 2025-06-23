/**
 * @file FLTPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "SwingTrajOpt.h"

using namespace geo_utils;

class FLTPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    SwingTrajOpt swing_traj_opt_;

public:
    FLTPlanner(SwingTrajPlannerConfig config,
               std::shared_ptr<BaseRobotInterface> robot_interface,
               std::shared_ptr<GridMapInterface> gridmap_interface);

    /**
     * @brief Search for a poly feasible trajectory
     *
     * @param[out] poly_traj
     * @param pose0
     * @param pose1
     * @param p0
     * @param p1
     * @param index
     * @param verbose
     * @return true
     * @return false
     */
    bool searchPolyTraj(std::vector<Point3D> &poly_traj,
                        const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                        const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                        uint index, bool verbose = true);

    std::shared_ptr<MincoTrajectory> getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                    double v_lift, double h_lift = 0.1);

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index) override;

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index) override;
};

class FLTCfgPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    SwingTrajOpt swing_traj_opt_;

public:
    FLTCfgPlanner(SwingTrajPlannerConfig config,
                  std::shared_ptr<BaseRobotInterface> robot_interface,
                  std::shared_ptr<GridMapInterface> gridmap_interface);

    void visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
                         std::vector<Point3D> cfg_poly_traj,
                         Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
                         int groupId = 1);

    /**
     * @brief Search for a poly feasible trajectory
     *
     * @param[out] poly_traj
     * @param pose0
     * @param pose1
     * @param p0
     * @param p1
     * @param index
     * @param verbose
     * @return true
     * @return false
     */
    bool searchPolyTraj(std::vector<Point3D> &poly_traj,
                        const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                        const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                        uint index, bool verbose = true);

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

    // Base Interface Override
    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index) override;

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index) override;

    /**
     * @brief Filter out unreachable points (using PITD)
     *
     * @param pose0
     * @param pose1
     * @param p0
     * @param[out] footholds
     * @param index
     * @return true
     * @return false
     */
    bool reachableCheckHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                            Eigen::Vector3d p0, uint index,
                            std::vector<Eigen::Vector3d> &footholds,
                            std::vector<bool> &reachable) override;
};