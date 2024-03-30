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

/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "fast_legged_planner/utils/Spline.h"

class SwingTrajPlanner
{
public:
    SwingTrajPlanner();
    ~SwingTrajPlanner();
    UniBSpline get_default_traj(Eigen::Vector3d &p0, Eigen::Vector3d &p1, double v_lift, double h_lift);
    bool opt_traj(TrajectoryBase &traj, int index);
};