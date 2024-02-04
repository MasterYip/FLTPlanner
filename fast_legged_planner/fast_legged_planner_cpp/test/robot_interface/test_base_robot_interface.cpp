/**
 * @file test_base_robot_interface.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fast_legged_planner/robot_interface/BaseRobotInterface.h"
#include "urdf_path.h"
#include <gtest/gtest.h>

TEST(InterfaceTest, test_base_robot_interface)
{
    BaseRobotInterface robot_interface(URDF_PATH, PKG_DIRS);
    // Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_interface.model_.nq);
    robot_interface.print_joints();
    robot_interface.print_frames();
}
