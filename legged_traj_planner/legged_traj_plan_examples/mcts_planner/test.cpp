#include <zobrist_hash.h>
#include <search_tree.h>
#include "ros/ros.h"
#include <hit_spider/hexapod_State.h>
#include <saveHashTable.h>
#include <contactPlannerInterface.h>
#include <userParameter.h>

#define SearchTimes 10000
#define N_Sliding 10000


int main(int argc, char *argv[])
{

    grid_map::GridMap map_;
    Parameters params(map_);
    // 打印参数
    params.printParameters();

    

    
    // 初始化机器人状态,并赋初值
    MDT::Pose robotPoseW = {0, 0, params.norminalTrunkHeight,  0,  0,  0*_PI_/6};
    MDT::Vector6b gaitToNow;
    gaitToNow << MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG;
    float moveDir = 0*_PI_/2;
    MDT::RobotState state_initial = HexapodParameter::initRobotState(robotPoseW, gaitToNow, moveDir, params);

    // 初始化路径点
    std::vector<Eigen::Vector3f> pathPnts;
    // 添加点到 pathPnts
    pathPnts.emplace_back(0, 0, 0.2f); // 使用emplace_back而非push_back可以减少额外的拷贝构造开销
    pathPnts.emplace_back(1, 0, 0.2f);
    pathPnts.emplace_back(2, 0, 0.2f);
    pathPnts.emplace_back(3, 0, 0.2f);
    pathPnts.emplace_back(4, 0, 0.2f);
    pathPnts.emplace_back(5, 0, 0.2f);
    pathPnts.emplace_back(6, 0, 0.2f);






    // 线性插值，每两个点之间插值10个点
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
    

    int count = 0;
    MDT::RobotState state_ = state_initial;



    double start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


    MDT::RobotState stateNext;
    CONTACT_PLANNER::pathTrackPlanner(state_,stateNext, interpolatedPoints, params, true, 1000);

    // for(int i = 0; i<1000; i++)
    // {
    //     auto results = PLANNING::getAvailableFootholds(state_, params.mapData);
    // }

    double end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::cout << "time:count:" << (end - start) / 1000 << "s" << std::endl;

    


    return 0;
}



