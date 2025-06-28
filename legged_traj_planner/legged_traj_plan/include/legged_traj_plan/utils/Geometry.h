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
#include "geometry_msgs/Pose.h"

pinocchio::SE3 XYZRPY2SE3(legged_traj_plan::hexapod_Base_Pose pose);

pinocchio::SE3 Pose2SE3(const geometry_msgs::Pose &pose);

legged_traj_plan::hexapod_Base_Pose SE32XYZRPY(const pinocchio::SE3 &se3);

geometry_msgs::Pose SE32Pose(const pinocchio::SE3 &se3);

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
        return 1 - sine_remap(std::max((dist - rmin) / (rmax - rmin),
                                       (center(2) - pos(2) - rmin) / (rmax - rmin)));
}

inline double inSphereCylinderSoft(const Eigen::Vector3d &pos, const Eigen::Vector3d &center,
                                   const Eigen::Vector3d &dir, double rmin, double rmax)
{
    Eigen::Vector3d dir_norm = dir;
    dir_norm.normalize();
    double dist_r = (pos - center).norm();
    double dist_axis = dir_norm.dot(pos - center);
    double dist_perp = dir_norm.cross(pos - center).norm();
    double dist = dist_axis > 0 ? dist_perp : dist_r;
    if (dist < rmin)
        return 1.0;
    else if (dist > rmax)
        return 0.0;
    else
        return 1 - sine_remap((dist - rmin) / (rmax - rmin));
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

/**
 * @brief Orthogonal disk randomize
 * @note Return a randomized vector \bar{r} given a normal vector \bar{n}, the \bar{r} is orthogonal to \bar{n}
 * @param normal
 * @param radius
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d orthogonalDiskRandomize(const Eigen::Vector3d &normal, double radius)
{
    Eigen::Vector3d random = Eigen::Vector3d::Random();
    random.normalize();
    Eigen::Vector3d tangent = random - random.dot(normal) * normal;
    return radius * tangent;
}