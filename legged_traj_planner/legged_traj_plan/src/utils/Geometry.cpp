#include "legged_traj_plan/utils/Geometry.h"

pinocchio::SE3 XYZRPY2SE3(legged_traj_plan::hexapod_Base_Pose pose)
{
    // RPY to Rotation Matrix
    Eigen::Matrix3d rotation_matrix = pinocchio::rpy::rpyToMatrix(
        Eigen::Vector3d(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw));

    // Create SE3 object with rotation matrix and translation vector
    pinocchio::SE3 se3(rotation_matrix, Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));

    return se3;
}

Eigen::Vector3d point_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Vector3d &pt)
{
    pinocchio::SE3 aMb = bMa.inverse();
    return aMb.translation() + aMb.rotation() * pt;
}

Eigen::Matrix3Xd point_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Matrix3Xd &pts)
{
    pinocchio::SE3 aMb = bMa.inverse();
    return aMb.translation().replicate(1, pts.cols()) + aMb.rotation() * pts;
}