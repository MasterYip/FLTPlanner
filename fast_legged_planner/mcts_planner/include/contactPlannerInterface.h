#ifndef CONTACT_PLANNER_INTERFACE_H
#define CONTACT_PLANNER_INTERFACE_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <myDataType.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

namespace CONTACT_PLANNER{
    /**
     * @brief 用于规划接触点的接口
     * @param currentState 当前机器人状态
     * @param pathPnts 需要跟踪的机体路径曲线关键点； 离散点之间的距离应该小于0.1m；否者规划器会失败； 可通过下面方面进行插值获得更密集的路径点
     *  // 线性插值，每两个点之间插值20个点
        std::vector<Eigen::Vector3f> interpolatedPoints;
        for (size_t i = 0; i < pathPnts.size() - 1; ++i) {
            Eigen::Vector3f start = pathPnts[i];
            Eigen::Vector3f end = pathPnts[i + 1];

            for (int j = 0; j <= 20; ++j) {
                float t = static_cast<float>(j) / 20.0;
                Eigen::Vector3f interpolatedPoint = start + t * (end - start);
                interpolatedPoints.push_back(interpolatedPoint);
            }
        }
     * @param mapData_ 地图数据
     * @param isMCTS 是否使用MCTS规划器; true: 使用MCTS规划器; false: 使用专家规划器
     * @return MDT::RobotState 规划后的机器人状态
     * 
     */
    MDT::RobotState pathTrackPlanner(const MDT::RobotState &currentState, const std::vector<Eigen::Vector3f>& pathPnts, const grid_map::GridMap& mapData_, const bool isMCTS);

}

#endif
