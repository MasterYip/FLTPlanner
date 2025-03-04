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
#include <collisionCheck.h>
#include <userParameter.h>
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

    MDT::RobotState getNextMCTSstate_forSim(const MDT::RobotState rState, const Parameters &params);

    MDT::RobotState getNextMCTSstateRotate_forSim(const MDT::RobotState rState, const Parameters &params);

    MDT::RobotState getTripleGaitPlanner(const MDT::RobotState rState, const Parameters &params, float stepLength);

    MDT::AvailableContactsInfo getAvailableFootholds(const MDT::RobotState &hexapodState, const Parameters &params);

    MDT::AvailableContactsInfo getAvailableFootholds_visual(const MDT::RobotState &hexapodState, const Parameters &params);

    std::vector<MDT::RobotState> getNextMCTSstateList_underConstrains_moreStates(const MDT::RobotState rState, const Parameters &params);

    MDT::RobotState getNextStateByExpert_underConstrain(const MDT::RobotState rState, const Parameters &params);

    MDT::VectorList swingLegFoot_position_visualize(const MDT::RobotState &rState, const Parameters &params);

    std::pair<float, float> getTargetYawAndMoveDir(MDT::RobotState currentState, const std::vector<Eigen::Vector3f>& pathPnts, const Parameters &params);

    int findTheNearestPathPointIndex(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts);

    int getTrackMovingDestination(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts);

    std::pair<std::vector<MDT::Vector6b>, std::vector<MDT::POINT>> getSupportListAndStepL_underConstrains(const MDT::RobotState hexapodState, const Parameters &params, bool isPrint);

    // 恢复规划器， 在实际部署中修正机器人落足点
    MDT::RobotState recoverPlanner(const MDT::RobotState currentState, const Parameters &params);

    std::vector<MDT::RobotState> swingLegFoot_position_lists(const MDT::RobotState &rState, const Parameters &params, float addL);

    std::pair<double, double> getMaxYawRotationAngle(MDT::RobotState currentState, const Parameters &params);
    
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
    MDT::RobotState getNextMCTSstateByExpert_rotate(const MDT::RobotState rState, const Parameters &params);
    ContactsInfos get_now_Feasible_foot_position(const MDT::Pose &pose, const Parameters &params);
    std::vector<MDT::RobotState> getNextMCTSstateByExpert_rotate_moreState(const MDT::RobotState rState, const Parameters &params);
}






#endif

