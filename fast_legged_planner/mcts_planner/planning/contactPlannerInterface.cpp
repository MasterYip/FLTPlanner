
#include <contactPlannerInterface.h> 
#include <planning.h> 
#include <search_tree.h>
#include <saveHashTable.h>

// #define N_Sliding 500  // 每次搜索的节点数
#define USE_MCTS_PLANNER  // 使用MCTS规划器；注释掉则使用专家规划器

namespace CONTACT_PLANNER{

    MDT::RobotState singleMCTS_planner(const MDT::RobotState &state_, const grid_map::GridMap &mapData, const std::vector<Eigen::Vector3f>& pathPnts, int search_nodes=100)
    {
        int oneStepSearchNodeNum = search_nodes; // 搜索一步使用搜索节点个数

        std::shared_ptr<TreeNode> startNode = std::make_shared<TreeNode>(state_, "&");

        startNode->expansion(mapData, pathPnts);
        /*
        Initialize HashTable
        */
        // std::srand(23);
        int processorN = 1;

        HashTable hsm(processorN, USER::key_element, USER::max_depth, USER::key_element.size());  // 每个线程都有一个hash表

        hsm.insert(Item(startNode->hashKey, startNode));

        double maxExtendedNode_Length = 0;
        std::string maxExtendedNode_hashKey;
        double maxSim_x = 0;
        double start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


        for (int i = 1; i < oneStepSearchNodeNum; ++i) {
            TreeNode_ptr node = hsm.search_table("&");

            // for (int j = 0; j < int(i / N_Sliding); ++j) {
            //     if (node->childNodes.empty()) {
            //         break;
            //     }
            //     node = hsm.search_table(node->findBestChild()->hashKey);
            // }

            // 这里包含完整的一轮选择,扩展,仿真和回溯
            while (true) {
                // std::cout << node->candidateNodes.size() << std::endl;
                if (!node->candidateNodes.empty()) { // 说明还有剩余的备选节点
                    int random_index = std::rand() % node->candidateNodes.size();
                    // 将其添加到儿子集合中
                    TreeNode_ptr newNode = node->addNode(random_index);

                    // 深度拷贝newChild
                    TreeNode_ptr newChild = std::make_shared<TreeNode>(*newNode);

                    // 扩展新的子节点
                    // 这里需要根据路径进行expansion！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！１
                    newChild->expansion(mapData, pathPnts);
                    


                    if (newChild->candidateNodes.empty()) {   // --------如果某个节点无备选子节点,那么分值设定为-10000
                        double score_ = -10000;
                        newChild->updateLocalNode(score_);
                        hsm.insert(Item(newChild->hashKey, newChild));
                        // break;
                    }
                    else{
                        // 进行仿真
                        // 这里需要根据路径进行simulation！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                        double simDistance = newChild->simulation(newChild->rState, mapData, pathPnts);
                        newChild->updateLocalSimDis(simDistance);

                        // 更新当前节点的分值
                        double score_ = newChild->calculateLocalScore(simDistance);
                        // std::cout << i << ", hashKey:" << newChild->hashKey << " maxExtendedNode_Length: " << newChild->rState.pose.x << " , maxSim_x: " << simDistance + newChild->rState.pose.x << std::endl;


                        newChild->updateLocalNode(score_);
                        // 添加进hash表中存储
                        hsm.insert(Item(newChild->hashKey, newChild));

                        // --------记录最值-----------------------------------------------------------
                        auto endPntIndex = PLANNING::findTheNearestPathPointIndex(Eigen::Vector3f(newChild->rState.pose.x, newChild->rState.pose.y, newChild->rState.pose.z), pathPnts);
                        float tmpDis = 0.0f;
                        for(int i = 0; i <= endPntIndex; ++i)
                        {
                            tmpDis += (pathPnts[i+1] - pathPnts[i]).norm();
                        }
                        float furtherDis = tmpDis;

                        if (furtherDis > maxExtendedNode_Length) {
                            maxExtendedNode_Length = furtherDis;
                            maxExtendedNode_hashKey = newChild->hashKey;
                            std::cout << i << ", hashKey:" << newChild->hashKey << " maxExtendedNode_Length: " << maxExtendedNode_Length << " , maxSim_x: " << maxSim_x << std::endl;
                        }
                        if (simDistance + tmpDis > maxSim_x) {
                            maxSim_x = simDistance + tmpDis;
                        }
                        // --------------------------------------------------------------------------


                    }


                    // 反向传播 
                    TreeNode_ptr local_node = newChild;
                    while (local_node->hashKey != "&") {
                        TreeNode_ptr parentNode = hsm.search_table(local_node->getParentKey());
                        parentNode->backpropagation_singleThread(local_node);
                        hsm.insert(Item(local_node->hashKey, local_node));
                        local_node = parentNode;
                    }
                    break;
                }

                node = node->selection_singleThread();
                node = hsm.search_table(node->hashKey);

                // --------如果某个节点无备选子节点,那么分值设定为-10000,然后反向传播,直至根节点停止---------------
                if (node->score < 0) {
                    std::cout << "node->score < 0: node->getParentKey():" << node->getParentKey() << std::endl;
                    node = hsm.search_table(node->getParentKey());
                    node->score = -10000;
                    // for(int i = 0; i < node->childNodes.size(); ++i)
                    // {
                    //     if(node->childNodes[i]->score > 0)
                    //     {
                    //         std::cout << "childNode: " << node->childNodes[i]->hashKey << " score: " << node->childNodes[i]->score << std::endl;

                    //         std::cout << "nodeChildScore:" << nodeSave->score << std::endl;
                    //         std::cout << "nodeChildScore2:" << nodeSave2->score << std::endl;
                    //         exit(1);
                    //     }
                    //     std::cout << "childNode: " << node->childNodes[i]->hashKey << " score: " << node->childNodes[i]->score << std::endl;
                    // }
                    // hsm.insert(Item(node->hashKey, node));
                    if (node->hashKey == "&") {

                        for(int i = 0; i < node->childNodes.size(); ++i)
                        {
                            std::cout << "childNode: " << node->childNodes[i]->hashKey << " score: " << node->childNodes[i]->score << std::endl;
                        }
                        // for (const auto& data : hsm.hashTable) {
                        //     if (data.second.value->score > 0) {
                        //         std::cout << data.second.key << " " << data.second.value->score << std::endl;
                        //     }
                        // }
                        // TreeNode_ptr furthestExtendedNode = hsm.search_table(maxExtendedNode_hashKey);
                        // std::cout << maxExtendedNode_Length << " " << maxSim_x << " " << furthestExtendedNode->hashKey << std::endl;
                        std::cout << hsm.hashTable.size() << std::endl;
                        break;
                    }
                    break;
                }
            }
        }
        double end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "expansion time: " << (end - start)/1000 << "s" <<std::endl;
        std::cout << "tableSize: " << hsm.hashTable.size() << std::endl;
        std::cout << "maxExtendedNode_hashKey: " << maxExtendedNode_hashKey << std::endl;

        std::vector<hit_spider::hexapod_State> stateList;
        TreeNode_ptr local_node = hsm.search_table(maxExtendedNode_hashKey);

        // 存放结果序列
        // hit_spider::hexapod_State stateLast = transRobotState(local_node->rState);
        // stateLast.remarks.data = "end_flag";
        // stateList.push_back(stateLast);
        if(maxExtendedNode_hashKey.size() < 2)
        {
            std::cout << "no solution" << std::endl;
            return state_;
        }

        while (true) {
            // std::cout << "x: " << local_node->rState.pose.x << std::endl;
            // local_node->drawLocalState(mapData_, workspaceL_);
            auto tmpNode = hsm.search_table(local_node->getParentKey());
            if(tmpNode->hashKey == "&")
            {
                return local_node->rState;
                break;
            }
            local_node = tmpNode; 
        }

    }



    MDT::RobotState pathTrackPlanner(const MDT::RobotState &currentState, const std::vector<Eigen::Vector3f>& pathPnts, const grid_map::GridMap& mapData_, const bool isMCTS, int search_nodes)
    {


        if(isMCTS)
        {
            // MDT::RobotState stateNext = singleMCTS_planner(currentState, mapData_,  pathPnts);
            MDT::RobotState stateNext = singleMCTS_planner(currentState, mapData_,  pathPnts, search_nodes);
            return stateNext;
        }
        else
        {
            // 根据路径和机器人当前位置,确定机器人前进方向
            MDT::RobotState state_ = currentState;
            auto targetAngle = PLANNING::getTargetYawAndMoveDir(state_, pathPnts);
            state_.moveDirection = targetAngle.second;
            // 如果旋转角度大于0.05,则只旋转
            float deltaYaw = state_.moveDirection - state_.pose.yaw;
            MDT::RobotState stateNext; 
            if(fabs (deltaYaw) > USER::onlyRotateThreshold)
            {
                stateNext  = PLANNING::getNextMCTSstateByExpert_rotate(state_, mapData_);
            }
            else
            {
                state_.pose.yaw = targetAngle.first;
                stateNext = PLANNING::getNextMCTSstateByExpert_underConstrain(state_, mapData_);
            }
            return stateNext;
        }
        
    }




}



