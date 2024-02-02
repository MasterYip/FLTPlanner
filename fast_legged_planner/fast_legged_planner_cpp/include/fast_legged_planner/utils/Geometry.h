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
#include <pinocchio/math/rpy.hpp>
/* internal project header files */
#include "fast_legged_planner/hexapod_Base_Pose.h"



pinocchio::SE3 XYZRPY2SE3(fast_legged_planner::hexapod_Base_Pose pose) {
    // RPY to Rotation Matrix
    Eigen::Matrix3d rotation_matrix = pinocchio::rpy::rpyToMatrix(
        Eigen::Vector3d(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw));
    
    // Create SE3 object with rotation matrix and translation vector
    pinocchio::SE3 se3(rotation_matrix, Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    
    return se3;
}
