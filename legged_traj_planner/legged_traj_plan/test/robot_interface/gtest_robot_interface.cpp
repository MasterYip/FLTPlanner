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

#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include <gtest/gtest.h>
#include "urdf_path.h"

using namespace std;

TEST(InterfaceTest, test_base_robot_interface)
{
    // BaseRobotInterface robot_interface(TEST_URDF_PATH, TEST_PKG_DIRS);
    BaseRobotInterface robot_interface(TEST_URDF_PATH);
    cout << "Joints" << endl;
    robot_interface.print_joints();
    cout << "Frames" << endl;
    robot_interface.print_frames();
}

TEST(InterfaceTest, test_elspider_air_interface)
{
    ElSpiderAirInterface robot_interface(TEST_URDF_PATH);
    // IK leg0
    Eigen::Vector3d pos(0.35350208, -0.22998902, -0.1360658);
    Eigen::Vector3d sol;
    Eigen::Vector3d exp_sol = Eigen::Vector3d(0, 0.01851, 0.00813);
    robot_interface.robot_kin.inverseKinConstraint(pos, sol, 0);
    cout << "Inverse Kinematics Solutions: " << endl;
    cout << sol.transpose() << endl;
    ASSERT_TRUE(sol.isApprox(exp_sol, 1e-2));
}
