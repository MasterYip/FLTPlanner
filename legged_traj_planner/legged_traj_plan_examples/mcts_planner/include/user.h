#ifndef _USER_H
#define _USER_H
#include <iostream>
#include <string>
#include <iostream>
#include <fstream> //文件操作
#include <sstream> //字符串操作,搭配fstream读取文件数据

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <map>

#include "config.h"

namespace USER
{
    void initRobotPinocchoModel(void);
    grid_map::GridMap init_grid_map(std::map<std::string, std::string> configMap);

    extern std::map<std::string, std::string> configMap;
    extern const std::string key_element; // HashKey 子元素
    extern const int max_depth;           // 最大深度
    extern const int simStepNum;          // 模拟步数
    extern grid_map::GridMap mapData;     // 地图数据
    extern const std::string availableFootholdLayerName;

    extern const bool COLLISION_CHECK;
    extern const bool IsMaxForceConstraint;
    extern const bool IsToruqeLimitConstraint;
    extern const bool IsVirtualLoss;
    extern const bool IsBestBP;

    extern const float SEARCH_TIME_LIMIT;
    extern const int JOB_FACTOR;

    const float onlyRotateThreshold = 0.05; // 旋转阈值
    const float norminalTrunkHeight = 0.26; // 默认机体高度

    const float nominal_y_shift = 0.10; // 默认落足点相对于死点向外的偏移量

    extern const float workspaceReductionDis;
    extern const float staticStabilityMarginThreshold;
    extern const float kinematicMargin;

    extern const float norminalFootStepLengthInWorkspace;

    extern const std::string elevationLayerName;
    extern const std::string normalVector_x_Name;
    extern const std::string normalVector_y_Name;
    extern const std::string normalVector_z_Name;

    // URDF 文件路径
    // std::string urdfPath = "/home/oem/aConstrainPlan/catkin_viewURDF/src/ROS-HexMini-Visual/el_mini/urdf/el_mini.urdf";

    extern const std::string robotURDF_Path;

    extern pinocchio::Model HexMini_PinoModel;

    extern pinocchio::Data HexMini_PinoModel_Data;

    extern const int GaitMode; // 步态模式

    // HexMini的URDF FrameName
    extern std::vector<std::string> FootName;
    extern std::vector<std::string> ShankName;
    extern std::vector<std::string> ThighName;

}
#endif
