#ifndef HEXAPOD_ROBOT_H
#define HEXAPOD_ROBOT_H
#include <userParameter.h>
#include <myDataType.h>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <iostream>
#include "constrains/util.hh"




namespace HexapodParameter {
    /**************自定义变量************************************************************************/
    extern const Eigen::Isometry3d TransMatrix_FixJi_Body_forHexMini[6];
    extern const Eigen::Isometry3d TransMatrix_Body_2_fixJi_forHexMini[6];
    extern const std::vector<MDT::Vector6b> initialSupportList;

    MDT::RobotState initRobotState(const MDT::Pose &robotPoseW,  MDT::Vector6b gaitToNow, float moveDirection, const Parameters &params);

   
}


















#endif

