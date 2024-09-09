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

/**
 * @brief Fit a plane to a set of points
 *
 * @param points
 * @param plane (a, b, c, d) such that ax + by + cz + d = 0
 * (normal vector is (a, b, c) and d is the distance from the origin to the plane
 * @return true
 * @return false
 */
bool plane_fitting(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &plane);

inline double sine_remap(double t)
{
    return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
}

// Exclude Checks

/**
 * @brief Check if a point is in the Z-axis cylinder
 *
 * @note The exclude cylinder is defined by a center and a radius,
 * the top height is infinite, the bottom height is at the center height minus the radius
 * @param pos
 * @param center
 * @param radius
 * @return true
 * @return false
 */
inline bool inZCylinder(const Eigen::Vector3d &pos, const Eigen::Vector3d &center, double radius)
{
    return (pos.head(2) - center.head(2)).norm() < radius && pos(2) > center(2) - radius;
}

inline double inZCylinderSoft(const Eigen::Vector3d &pos, const Eigen::Vector3d &center,
                              double rmin, double rmax)
{
    double dist = (pos.head(2) - center.head(2)).norm();
    if (dist < rmin && pos(2) > center(2) - rmin)
        return 1.0;
    else if (dist > rmax || pos(2) < center(2) - rmax)
        return 0.0;
    else
        return 1 - std::max(sine_remap((dist - rmin) / (rmax - rmin)),
                            sine_remap((center(2) - pos(2) - rmin) / (rmax - rmin)));
}

inline bool inSphere(const Eigen::Vector3d &pos, const Eigen::Vector3d &center, double radius)
{
    return (pos - center).norm() < radius;
}

inline double inSphereSoft(const Eigen::Vector3d &pos, const Eigen::Vector3d &center,
                           double rmin, double rmax)
{
    double dist = (pos - center).norm();
    if (dist < rmin)
        return 1.0;
    else if (dist > rmax)
        return 0.0;
    else
        return 1 - sine_remap((dist - rmin) / (rmax - rmin));
}