/**
 * @file SwingTrajPlannerRRT.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-07-14
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
#include "legged_traj_plan/swing_leg_planner_rrt/SwingTrajOptRRT.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

class SwingTrajPlannerRRT : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    SwingTrajOptRRT swing_traj_opt_;

public:
    SwingTrajPlannerRRT(SwingTrajPlannerConfig config,
                        std::shared_ptr<ElSpiderAirInterface> robot_interface,
                        std::shared_ptr<GridMapInterface> gridmap_interface);
    ~SwingTrajPlannerRRT() = default;

    std::shared_ptr<TrajectoryBase> getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                uint index);

    bool optTraj(std::shared_ptr<TrajectoryBase> &traj,
                 const pinocchio::SE3 &pose0,
                 const pinocchio::SE3 &pose1,
                 int index);
};