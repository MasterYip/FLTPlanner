
#include <contactPlannerInterface.h>
#include <planning.h>
#include <search_tree.h>
#include <saveHashTable.h>
#include "constrains/kinematics_constrain.hh" //运动学约束
#include <chrono>

// #define N_Sliding 500  // 每次搜索的节点数
#define USE_MCTS_PLANNER // 使用MCTS规划器；注释掉则使用专家规划器

namespace CONTACT_PLANNER
{

    MDT::RobotState singleMCTS_planner(const MDT::RobotState &state_,
                                       const Parameters &params,
                                       const std::vector<Eigen::Vector3f> &pathPnts,
                                       int search_nodes = 100)
    {
        int oneStepSearchNodeNum = search_nodes; // 搜索一步使用搜索节点个数

        std::shared_ptr<TreeNode> startNode = std::make_shared<TreeNode>(state_, "&");

        startNode->expansion(params, pathPnts);
        if (startNode->candidateNodes.empty())
        {
            throw std::runtime_error("startNode->candidateNodes.empty()");
        }

        // Initialize HashTable
        int processorN = 1;

        HashTable hsm(processorN, params.key_element, params.max_depth, params.key_element.size()); // 每个线程都有一个hash表

        hsm.insert(Item(startNode->hashKey, startNode));

        double maxExtendedNode_Length = 0;
        float maxExtendedNode_score = 0;
        std::string maxExtendedNode_hashKey;
        double maxSim_x = 0;
        double start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        for (int jjj = 1; jjj < oneStepSearchNodeNum; ++jjj)
        {
            TreeNode_ptr node = hsm.search_table("&");

            // 这里包含完整的一轮选择,扩展,仿真和回溯
            while (true)
            {
                // std::cout << node->candidateNodes.size() << std::endl;
                if (!node->candidateNodes.empty())
                { // 说明还有剩余的备选节点
                    int random_index = std::rand() % node->candidateNodes.size();
                    // 将其添加到儿子集合中
                    TreeNode_ptr newNode = node->addNode(random_index, params);

                    // 深度拷贝newChild
                    TreeNode_ptr newChild = std::make_shared<TreeNode>(*newNode);
                    float deltaX_ = node->rState.pose.x - newChild->rState.pose.x;
                    float deltaY_ = node->rState.pose.y - newChild->rState.pose.y;
                    float disToPar = sqrt(deltaX_ * deltaX_ + deltaY_ * deltaY_);
                    float disFeetMove = 0.0f;
                    int SwingLegNum = 0;
                    for (int i = 0; i < 6; ++i)
                    {
                        if (newChild->rState.gaitToNow[i] == MDT::SWING_FLAG)
                        {
                            SwingLegNum++;
                            disFeetMove += (newChild->rState.feetPosition[i] - node->rState.feetPosition[i]).norm();
                        }
                    }

                    newChild->disToParents = disToPar + disFeetMove / float(SwingLegNum);
                    // 扩展新的子节点
                    // 这里需要根据路径进行expansion！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！１
                    newChild->expansion(params, pathPnts);

                    if (newChild->candidateNodes.empty())
                    { // --------如果某个节点无备选子节点,那么分值设定为-10000
                        double score_ = -10000;
                        newChild->updateLocalNode(score_);
                        hsm.insert(Item(newChild->hashKey, newChild));

                        // // 防止子节点都无备选节点时，返回结果报错
                        // if ((hsm.hashTable.size() == 2))
                        // {
                        //     maxExtendedNode_hashKey = newChild->hashKey;
                        // }
                        // break;
                    }
                    else
                    {
                        // 进行仿真
                        // 这里需要根据路径进行simulation！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                        double simDistance = newChild->simulation(newChild->rState, params, pathPnts);
                        newChild->updateLocalSimDis(simDistance);

                        // 更新当前节点的分值
                        double score_ = newChild->calculateLocalScore(simDistance, params);
                        // std::cout << i << ", hashKey:" << newChild->hashKey << " maxExtendedNode_Length: " << newChild->rState.pose.x << " , maxSim_x: " << simDistance + newChild->rState.pose.x << std::endl;

                        newChild->updateLocalNode(score_);
                        // 添加进hash表中存储
                        hsm.insert(Item(newChild->hashKey, newChild));

                        // --------记录最值-----------------------------------------------------------
                        auto endPntIndex = PLANNING::findTheNearestPathPointIndex(Eigen::Vector3f(newChild->rState.pose.x, newChild->rState.pose.y, newChild->rState.pose.z), pathPnts);
                        float tmpDis = 0.0f;
                        for (int i = 0; i <= endPntIndex; ++i)
                        {
                            tmpDis += (pathPnts[i + 1] - pathPnts[i]).norm();
                        }
                        float furtherDis = tmpDis;

                        if ((furtherDis > maxExtendedNode_Length) || (furtherDis == maxExtendedNode_Length && score_ > maxExtendedNode_score))
                        {
                            maxExtendedNode_score = score_;
                            maxExtendedNode_Length = furtherDis;
                            maxExtendedNode_hashKey = newChild->hashKey;
                            std::cout << jjj << ", hashKey:" << newChild->hashKey << " maxExtendedNode_Length: " << maxExtendedNode_Length << " , maxSim_x: " << maxSim_x << std::endl;
                        }

                        if (simDistance + tmpDis > maxSim_x)
                        {
                            maxSim_x = simDistance + tmpDis;
                        }

                        // stop in advance
                        if (newChild->hashKey.size() > 5)
                        {
                            jjj = oneStepSearchNodeNum + 1;
                        }
                        // --------------------------------------------------------------------------
                    }

                    // 反向传播
                    TreeNode_ptr local_node = newChild;
                    while (local_node->hashKey != "&")
                    {
                        TreeNode_ptr parentNode = hsm.search_table(local_node->getParentKey());
                        parentNode->backpropagation_singleThread(local_node, params);
                        hsm.insert(Item(local_node->hashKey, local_node));
                        local_node = parentNode;
                    }
                    break;
                }

                node = node->selection_singleThread();
                node = hsm.search_table(node->hashKey);

                // --------如果某个节点无备选子节点,那么分值设定为-10000,然后反向传播,直至根节点停止---------------
                if (node->score < 0)
                {
                    // std::cout << "node->score < 0: node->getParentKey():" << node->getParentKey() << std::endl;
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
                    if (node->hashKey == "&")
                    {

                        // for (int i = 0; i < node->childNodes.size(); ++i)
                        // {
                        //     std::cout << "childNode: " << node->childNodes[i]->hashKey << " score: " << node->childNodes[i]->score << std::endl;
                        // }

                        // std::cout << hsm.hashTable.size() << std::endl;
                        break;
                    }
                    break;
                }
            }
        }
        double end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "expansion time: " << (end - start) / 1000 << "s" << std::endl;
        std::cout << "tableSize: " << hsm.hashTable.size() << std::endl;
        std::cout << "maxExtendedNode_hashKey: " << maxExtendedNode_hashKey << std::endl;

        std::vector<hit_spider::hexapod_State> stateList;
        TreeNode_ptr local_node = hsm.search_table(maxExtendedNode_hashKey);

        // 存放结果序列
        // hit_spider::hexapod_State stateLast = transRobotState(local_node->rState);
        // stateLast.remarks.data = "end_flag";
        // stateList.push_back(stateLast);
        if (maxExtendedNode_hashKey.size() < 2)
        {
            throw std::runtime_error("No solution");
        }

        while (true)
        {
            // std::cout << "x: " << local_node->rState.pose.x << std::endl;
            // local_node->drawLocalState(params.mapData_, workspaceL_);
            auto tmpNode = hsm.search_table(local_node->getParentKey());
            if (tmpNode->hashKey == "&")
            {
                return local_node->rState;
                break;
            }
            local_node = tmpNode;
        }
    }

    bool MCTS_planner(const MDT::RobotState &state_,
                      std::vector<MDT::RobotState> &plannedStates,
                      const Parameters &params,
                      const std::vector<Eigen::Vector3f> &pathPnts,
                      int search_nodes = 100)
    {
        int oneStepSearchNodeNum = search_nodes; // 搜索一步使用搜索节点个数
        std::shared_ptr<TreeNode> startNode = std::make_shared<TreeNode>(state_, "&");
        startNode->expansion(params, pathPnts);
        if (startNode->candidateNodes.empty())
        {
            throw std::runtime_error("startNode->candidateNodes.empty()");
        }

        // Initialize HashTable
        int processorN = 1;
        HashTable hsm(processorN, params.key_element, params.max_depth, params.key_element.size()); // 每个线程都有一个hash表
        hsm.insert(Item(startNode->hashKey, startNode));
        double maxExtendedNode_Length = 0;
        std::string maxExtendedNode_hashKey;
        double maxSim_x = 0;
        double start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        for (int jjj = 1; jjj < oneStepSearchNodeNum; ++jjj)
        {
            TreeNode_ptr node = hsm.search_table("&");

            // 这里包含完整的一轮选择,扩展,仿真和回溯
            while (true)
            {
                // std::cout << node->candidateNodes.size() << std::endl;
                if (!node->candidateNodes.empty())
                { // 说明还有剩余的备选节点
                    int random_index = std::rand() % node->candidateNodes.size();
                    // 将其添加到儿子集合中
                    TreeNode_ptr newNode = node->addNode(random_index, params);
                    // 深度拷贝newChild
                    TreeNode_ptr newChild = std::make_shared<TreeNode>(*newNode);
                    float deltaX_ = node->rState.pose.x - newChild->rState.pose.x;
                    float deltaY_ = node->rState.pose.y - newChild->rState.pose.y;
                    float disToPar = sqrt(deltaX_ * deltaX_ + deltaY_ * deltaY_);
                    float disFeetMove = 0.0f;
                    int SwingLegNum = 0;
                    for (int i = 0; i < 6; ++i)
                    {
                        if (newChild->rState.gaitToNow[i] == MDT::SWING_FLAG)
                        {
                            SwingLegNum++;
                            disFeetMove += (newChild->rState.feetPosition[i] - node->rState.feetPosition[i]).norm();
                        }
                    }

                    newChild->disToParents = disToPar + disFeetMove / float(SwingLegNum);
                    // 扩展新的子节点
                    // 这里需要根据路径进行expansion！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！１
                    newChild->expansion(params, pathPnts);
                    if (newChild->candidateNodes.empty())
                    { // --------如果某个节点无备选子节点,那么分值设定为-10000
                        double score_ = -10000;
                        newChild->updateLocalNode(score_);
                        hsm.insert(Item(newChild->hashKey, newChild));
                        // break;
                    }
                    else
                    {
                        // 进行仿真
                        // 这里需要根据路径进行simulation！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                        double simDistance = newChild->simulation(newChild->rState, params, pathPnts);
                        newChild->updateLocalSimDis(simDistance);
                        // 更新当前节点的分值
                        double score_ = newChild->calculateLocalScore(simDistance, params);
                        newChild->updateLocalNode(score_);
                        // 添加进hash表中存储
                        hsm.insert(Item(newChild->hashKey, newChild));
                        // --------记录最值-----------------------------------------------------------
                        auto endPntIndex = PLANNING::findTheNearestPathPointIndex(Eigen::Vector3f(newChild->rState.pose.x, newChild->rState.pose.y, newChild->rState.pose.z), pathPnts);
                        float tmpDis = 0.0f;
                        for (int i = 0; i <= endPntIndex; ++i)
                        {
                            tmpDis += (pathPnts[i + 1] - pathPnts[i]).norm();
                        }
                        float furtherDis = tmpDis;
                        if (furtherDis > maxExtendedNode_Length)
                        {
                            maxExtendedNode_Length = furtherDis;
                            maxExtendedNode_hashKey = newChild->hashKey;
                            std::cout << jjj << ", hashKey:" << newChild->hashKey << " maxExtendedNode_Length: " << maxExtendedNode_Length << " , maxSim_x: " << maxSim_x << std::endl;
                        }
                        if (simDistance + tmpDis > maxSim_x)
                        {
                            maxSim_x = simDistance + tmpDis;
                        }
                    }
                    // 反向传播
                    TreeNode_ptr local_node = newChild;
                    while (local_node->hashKey != "&")
                    {
                        TreeNode_ptr parentNode = hsm.search_table(local_node->getParentKey());
                        parentNode->backpropagation_singleThread(local_node, params);
                        hsm.insert(Item(local_node->hashKey, local_node));
                        local_node = parentNode;
                    }
                    break;
                }
                node = node->selection_singleThread();
                node = hsm.search_table(node->hashKey);
                // --------如果某个节点无备选子节点,那么分值设定为-10000,然后反向传播,直至根节点停止---------------
                if (node->score < 0)
                {
                    std::cout << "node->score < 0: node->getParentKey():" << node->getParentKey() << std::endl;
                    node = hsm.search_table(node->getParentKey());
                    node->score = -10000;
                    if (node->hashKey == "&")
                    {

                        for (int i = 0; i < node->childNodes.size(); ++i)
                        {
                            std::cout << "childNode: " << node->childNodes[i]->hashKey << " score: " << node->childNodes[i]->score << std::endl;
                        }
                        std::cout << hsm.hashTable.size() << std::endl;
                        break;
                    }
                    break;
                }
            }
        }
        double end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "expansion time: " << (end - start) / 1000 << "s" << std::endl;
        std::cout << "tableSize: " << hsm.hashTable.size() << std::endl;
        std::cout << "maxExtendedNode_hashKey: " << maxExtendedNode_hashKey << std::endl;

        std::vector<hit_spider::hexapod_State> stateList;
        TreeNode_ptr local_node = hsm.search_table(maxExtendedNode_hashKey);

        if (maxExtendedNode_hashKey.size() < 2)
        {
            throw std::runtime_error("No solution");
        }
        while (true)
        {
            auto tmpNode = hsm.search_table(local_node->getParentKey());
            plannedStates.insert(plannedStates.begin(), local_node->rState);
            if (tmpNode->hashKey == "&")
            {
                break;
            }
            local_node = tmpNode;
        }
        return true;
    }

    bool pathTrackPlanner(const MDT::RobotState &currentState,
                          std::vector<MDT::RobotState> &nextState,
                          const std::vector<Eigen::Vector3f> &pathPnts,
                          const Parameters &params,
                          int search_nodes)
    {
        try
        {
            return MCTS_planner(currentState, nextState, params, pathPnts, search_nodes);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            PLANNING::getSupportListAndStepL_underConstrains(currentState, params, true);

            // 检查是否超出工作空间
            for (int legIndex = 0; legIndex < 6; ++legIndex)
            {
                if (currentState.gaitToNow[legIndex] == MDT::SUPPORT_FLAG)
                {
                    Eigen::Vector3d footW;
                    footW << currentState.feetPosition[legIndex].x(), currentState.feetPosition[legIndex].y(), currentState.feetPosition[legIndex].z();
                    auto Ab = Robot_State_Transition::get_oneLegKinematics_con_cog_foot(currentState.pose, legIndex, false, params);
                    if (!Robot_State_Transition::isInConvex(Ab.first, Ab.second, footW))
                    {
                        std::cout << "leg: " << legIndex << " 超出工作空间" << std::endl;
                    }
                }
            }

            std::cout << "MCTS规划失败, 启动恢复规划器" << std::endl;
            // Delay
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            nextState.emplace_back(CONTACT_PLANNER::recoverPlannerInterface(currentState, params));
            return true;
        }
    }

    bool pathTrackPlanner(const MDT::RobotState &currentState,
                          MDT::RobotState &nextState,
                          const std::vector<Eigen::Vector3f> &pathPnts,
                          const Parameters &params,
                          const bool isMCTS,
                          int search_nodes)
    {
        if (isMCTS)
        {
            try
            {
                nextState = singleMCTS_planner(currentState, params, pathPnts, search_nodes);
                return true;
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                PLANNING::getSupportListAndStepL_underConstrains(currentState, params, true);

                // 检查是否超出工作空间
                for (int legIndex = 0; legIndex < 6; ++legIndex)
                {
                    if (currentState.gaitToNow[legIndex] == MDT::SUPPORT_FLAG)
                    {
                        Eigen::Vector3d footW;
                        footW << currentState.feetPosition[legIndex].x(), currentState.feetPosition[legIndex].y(), currentState.feetPosition[legIndex].z();
                        auto Ab = Robot_State_Transition::get_oneLegKinematics_con_cog_foot(currentState.pose, legIndex, false, params);
                        if (!Robot_State_Transition::isInConvex(Ab.first, Ab.second, footW))
                        {
                            std::cout << "leg: " << legIndex << " 超出工作空间" << std::endl;
                        }
                    }
                }

                std::cout << "MCTS规划失败, 启动恢复规划器" << std::endl;
                // Delay
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                nextState = CONTACT_PLANNER::recoverPlannerInterface(currentState, params);
                return true;

                // getchar();
                // try
                // {
                //     MDT::Vector6b gaitToNow;
                //     auto newState = currentState;
                //     gaitToNow << MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG,MDT::SUPPORT_FLAG;
                //     newState.gaitToNow = gaitToNow;
                //     nextState = singleMCTS_planner(newState, mapData_, pathPnts, search_nodes);
                //     return true;
            // }
            // catch (const std::exception &e)
            // {
                //     return false;
                // }
            }
        }
        else
        {
            // 根据路径和机器人当前位置,确定机器人前进方向
            MDT::RobotState state_ = currentState;
            auto targetAngle = PLANNING::getTargetYawAndMoveDir(state_, pathPnts, params);
            state_.moveDirection = targetAngle.second;
            // FIXME: 如果旋转角度大于0.05,则只旋转
            float deltaYaw = state_.moveDirection - state_.pose.yaw;
            if (fabs(deltaYaw) > params.onlyRotateThreshold)
            {
                nextState = PLANNING::getNextMCTSstateByExpert_rotate(state_, params);
            }
            else
            {
                state_.pose.yaw = targetAngle.first;
                nextState = PLANNING::getNextStateByExpert_underConstrain(state_, params);
            }
        }
    }

    MDT::RobotState tripleGaitPlanner(const MDT::RobotState rState, const Parameters &params, float stepLength)
    {
        return PLANNING::getTripleGaitPlanner(rState, params, stepLength);
    }
    // MDT::RobotState

    MDT::RobotState recoverPlannerInterface(const MDT::RobotState rState, const Parameters &params)
    {
        return PLANNING::recoverPlanner(rState, params);
    }

}
