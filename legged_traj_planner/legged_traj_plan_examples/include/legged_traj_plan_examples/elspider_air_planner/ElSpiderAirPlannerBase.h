/**
 * @file ElSpiderAirPlannerBase.h
 * @author Master Yip (2205029492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h" // Should be included first (pinocchio)
#include "legged_traj_search/utils/gcs_visualizer.hpp"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

struct PlannerConfig
{
    std::string cmdvelFrame;
    
}
