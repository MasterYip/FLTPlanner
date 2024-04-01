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
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"

class SwingTrajPlanner
{
private:
    BaseRobotInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;

public:
    SwingTrajPlanner(BaseRobotInterface &robot_interface, GridMapInterface &gridmap_interface);
    ~SwingTrajPlanner() = default;
    std::shared_ptr<TrajectoryBase> getDefaultTraj(Eigen::Vector3d &p0, Eigen::Vector3d &p1, double v_lift, double h_lift = 0.1);
    std::shared_ptr<TrajectoryBase> getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                double v_lift, uint index);
    bool opt_traj(std::shared_ptr<TrajectoryBase> traj, int index);
};