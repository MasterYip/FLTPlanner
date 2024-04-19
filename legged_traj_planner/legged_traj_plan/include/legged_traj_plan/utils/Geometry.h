/**
 * @file Geometry.h
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
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/math/rpy.hpp>
/* internal project header files */
#include "legged_traj_plan/hexapod_Base_Pose.h"

pinocchio::SE3 XYZRPY2SE3(legged_traj_plan::hexapod_Base_Pose pose);

Eigen::Vector3d point_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Vector3d &pt);

Eigen::Vector3d vec_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Vector3d &vec);

Eigen::Matrix3Xd points_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Matrix3Xd &pts);

pinocchio::SE3 poseLinearInterp(pinocchio::SE3 pose0, pinocchio::SE3 pose1, double t);