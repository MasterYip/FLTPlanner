#ifndef PLANNING_H
#define PLANNING_H

#include <myDataType.h>
#include <vector>
#include <geometryFun.h>
// #include <grid_map_ros/grid_map_ros.hpp> //grid_map地图
// #include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <HexapodParameter.h>
#include <user.h>
#include <collisionCheck.h>

namespace PLANNING{

    struct ContactsInfos{
        MatrixX3 position;
        MatrixX3 normalVector;
        VectorX  frcitionMu;
        VectorX  maxNormalF;
    }; 



    /**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_forSim(const MDT::RobotState rState, const grid_map::GridMap &mapData);


    MDT::AvailableContactsInfo getAvailableFootholds(const MDT::RobotState &hexapodState, const grid_map::GridMap &mapData);

    std::vector<MDT::RobotState> getNextMCTSstateList_underConstrains_moreStates(const MDT::RobotState rState, const grid_map::GridMap &mapData);

    MDT::RobotState getNextMCTSstateByExpert_underConstrain(const MDT::RobotState rState, const grid_map::GridMap &mapData);

    MDT::VectorList swingLegFoot_position_visualize(const MDT::RobotState &rState, const grid_map::GridMap &mapData);

    std::pair<float, float> getTargetYawAndMoveDir(MDT::RobotState currentState, const std::vector<Eigen::Vector3f>& pathPnts);

    int findTheNearestPathPointIndex(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts);

    int getTrackMovingDestination(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts);

    std::pair<double, double> getMaxYawRotationAngle(MDT::RobotState currentState);
    
    /**
     * \brief 计算两个状态下机器人沿着path的距离
     * \param startState 起始状态
     * \param endState 终止状态
     * \param pathPnts 路径点
     * \return 返回两个状态下机器人沿着path的距离
     * 
    */
    float getTwoStateDistanceAlongPath(const MDT::RobotState &startState, const MDT::RobotState &endState, const std::vector<Eigen::Vector3f>& pathPnts);

    /**
     * \brief 纯旋转, 未考虑各种约束
     * 
    */
    MDT::RobotState getNextMCTSstateByExpert_rotate(const MDT::RobotState rState, const grid_map::GridMap &mapData);

    std::vector<MDT::RobotState> getNextMCTSstateByExpert_rotate_moreState(const MDT::RobotState rState, const grid_map::GridMap &mapData);
}






#endif

