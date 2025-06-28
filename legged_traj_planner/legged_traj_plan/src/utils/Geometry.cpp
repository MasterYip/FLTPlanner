#include "legged_traj_plan/utils/Geometry.h"

/**
 * @brief Pose linear interpolation
 * 
 * @param pose0 
 * @param pose1 
 * @param t [0, 1]
 * @return pinocchio::SE3 
 */
pinocchio::SE3 poseLinearInterp(pinocchio::SE3 pose0, pinocchio::SE3 pose1, double t)
{
    if (pose0.isApprox(pose1))
    {
        return pose0;
    }
    pinocchio::Motion err = pinocchio::log6(pose0.actInv(pose1));
    pinocchio::SE3 interp = pose0.act(pinocchio::exp6(err * t));
    return interp;
}

pinocchio::SE3 XYZRPY2SE3(legged_traj_plan::hexapod_Base_Pose pose)
{
    // RPY to Rotation Matrix
    Eigen::Matrix3d rotation_matrix = pinocchio::rpy::rpyToMatrix(
        Eigen::Vector3d(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw));

    // Create SE3 object with rotation matrix and translation vector
    pinocchio::SE3 se3(rotation_matrix, Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));

    return se3;
}

pinocchio::SE3 Pose2SE3(const geometry_msgs::Pose &pose)
{
    return pinocchio::SE3(
        Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
}

legged_traj_plan::hexapod_Base_Pose SE32XYZRPY(const pinocchio::SE3 &se3)
{
    legged_traj_plan::hexapod_Base_Pose pose;
    
    // Extract translation (XYZ)
    Eigen::Vector3d translation = se3.translation();
    pose.position.x = translation[0];
    pose.position.y = translation[1];
    pose.position.z = translation[2];
    
    // Extract rotation matrix and convert to RPY
    Eigen::Matrix3d rotation_matrix = se3.rotation();
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(rotation_matrix);
    pose.orientation.roll = rpy[0];
    pose.orientation.pitch = rpy[1];
    pose.orientation.yaw = rpy[2];
    
    return pose;
}

geometry_msgs::Pose SE32Pose(const pinocchio::SE3 &se3)
{
    geometry_msgs::Pose pose;
    
    // Extract translation (XYZ)
    Eigen::Vector3d translation = se3.translation();
    pose.position.x = translation[0];
    pose.position.y = translation[1];
    pose.position.z = translation[2];
    
    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation_matrix = se3.rotation();
    Eigen::Quaterniond quaternion(rotation_matrix);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    
    return pose;
}



/**
 * @brief Transform a point from frame b to frame a (or apply a SE3 transformation to a point)
 *
 * @param bMa
 * @param pt
 * @return Eigen::Vector3d
 */
// FIXME: .inverse() is used twice somewhere in the code, performance can be improved.
Eigen::Vector3d point_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Vector3d &pt)
{
    pinocchio::SE3 aMb = bMa.inverse();
    return aMb.translation() + aMb.rotation() * pt;
}

Eigen::Vector3d vec_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Vector3d &vec)
{
    return bMa.inverse().rotation() * vec;
}

Eigen::Matrix3Xd points_SE3Act(const pinocchio::SE3 &bMa, const Eigen::Matrix3Xd &pts)
{
    pinocchio::SE3 aMb = bMa.inverse();
    return aMb.translation().replicate(1, pts.cols()) + aMb.rotation() * pts;
}

/**
 * @brief Fit a plane to a set of points
 *
 * @param points
 * @param plane (a, b, c) such that ax + by + c = z
 * (normal vector is (-a, -b, 1).normal
 * @return true
 * @return false
 */
bool plane_fitting(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &plane)
{
    if (points.size() < 3)
    {
        std::cerr << "At least 3 points are required for plane fitting" << std::endl;
        return false;
    }
    Eigen::MatrixX3d Pts(points.size(), 3);
    for (size_t i = 0; i < points.size(); i++)
    {
        Pts.row(i) << points[i].transpose();
    }
    Eigen::VectorXd X = Pts.col(0);
    Eigen::VectorXd Y = Pts.col(1);
    Eigen::VectorXd Z = Pts.col(2);
    Eigen::VectorXd I = Eigen::VectorXd::Ones(points.size());
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    A << X.transpose() * X, X.transpose() * Y, X.sum(),
        Y.transpose() * X, Y.transpose() * Y, Y.sum(),
        X.sum(), Y.sum(), points.size();
    b << X.transpose() * Z, Y.transpose() * Z, Z.sum();
    plane << A.inverse() * b;
    return true;
}