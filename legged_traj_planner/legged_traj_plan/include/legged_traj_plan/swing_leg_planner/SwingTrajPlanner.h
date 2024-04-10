/**
 * @file SwingTrajPlanner.h
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
#include <memory>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include "legged_traj_plan/swing_leg_planner/SwingTrajOpt.h"
using namespace geo_utils;

class SwingTrajPlanner
{
private:
    ElSpiderAirInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;
    ros::NodeHandle nh_;
    GCSVisualizer visualizer_;
    SwingTrajOpt swing_traj_opt_;

public:
    SwingTrajPlanner(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface);
    ~SwingTrajPlanner() = default;

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

    std::shared_ptr<MincoTrajectory> getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                       const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                                                       double v_lift, double h_lift = 0.1);

    std::shared_ptr<MincoTrajectory> getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                 double v_lift, uint index);

    bool getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                        pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                        Eigen::Vector3d p0, Eigen::Vector3d p1,
                        uint index);
    std::shared_ptr<MincoTrajectory> getCfgInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    double v_lift, uint index);

    bool opt_traj(std::shared_ptr<TrajectoryBase> &traj,
                  const pinocchio::SE3 &pose0,
                  const pinocchio::SE3 &pose1,
                  int index);

    ElSpiderAirInterface &getRobotInterface()
    {
        return robot_interface_;
    }
};