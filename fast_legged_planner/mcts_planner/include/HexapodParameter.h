#ifndef HEXAPOD_ROBOT_H
#define HEXAPOD_ROBOT_H

#include <myDataType.h>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <user.h>
#include <iostream>
#include "constrains/util.hh"





namespace HexapodParameter {
    /**************自定义变量************************************************************************/
    extern const float body_fixJi_theta[6];
    extern const Eigen::Isometry3d TransMatrix_FixJi_Body_forHexMini[6];
    extern const Eigen::Isometry3d TransMatrix_Body_2_fixJi_forHexMini[6];
    extern const std::vector<MDT::Vector6b> initialSupportList;
    extern const MDT::POINT norminalFoothold_B[6];

    MDT::RobotState initRobotState(const MDT::Pose &robotPoseW,  MDT::Vector6b gaitToNow, float moveDirection);
    bool support_leg_inverse_kin(const MDT::Pose &Wolrd_BasePose, const MDT::FeetPositions &World_feet, 
        const std::vector<int> &support_leg, MatrixX3 &joints_output);
   
}


















#endif

