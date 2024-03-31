#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <myDataType.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

namespace COLLISION_CHECK{

    bool isSwingLegCollision(Eigen::Vector3d startP, Eigen::Vector3d endP, const grid_map::GridMap &mapData, int legNum,
        Eigen::Vector3d &BodyPositionBegin,  Eigen::Vector3d &BodyPositionEnd, Eigen::Vector3d &BodyPoseBegin, Eigen::Vector3d &BodyPoseEnd);

    std::vector<Eigen::Vector3d> getLegKeyPoints(Eigen::Isometry3d T_W_B, Eigen::Vector3d EndEffectorP_W, int legNum);

    bool isCollision_onePoint(Eigen::Vector3d pnt, const grid_map::GridMap &mapData);

    // 只考虑足端轨迹的碰撞检测
    bool isFootTrajectoryCollision(Eigen::Vector3d startP, Eigen::Vector3d endP, const grid_map::GridMap &mapData);
}

#endif
