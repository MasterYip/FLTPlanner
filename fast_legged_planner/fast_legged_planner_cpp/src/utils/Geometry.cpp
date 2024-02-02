#include "fast_legged_planner/utils/Geometry.h"

pinocchio::SE3 XYZRPY2SE3(fast_legged_planner::hexapod_Base_Pose pose)
{
    // RPY to Rotation Matrix
    Eigen::Matrix3d rotation_matrix = pinocchio::rpy::rpyToMatrix(
        Eigen::Vector3d(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw));

    // Create SE3 object with rotation matrix and translation vector
    pinocchio::SE3 se3(rotation_matrix, Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));

    return se3;
}