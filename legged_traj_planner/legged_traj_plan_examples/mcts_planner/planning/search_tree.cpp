#include <search_tree.h>
#include <cmath>

/**
 * 根据UCB算法选择子节点, 注意num_thread_visited是用于构建虚拟loss的
 * @brief 选择子节点
 * @return TreeNode_ptr 选择的子节点
 */
TreeNode_ptr TreeNode::selection(bool isVirtualLoss)
{
    assert(!this->childNodes.empty() && "Child nodes should not be empty!");

    // 如果该节点的深度小于4,则不使用虚拟loss, 允许多线程同时访问
    if (this->hashKey.size() < 4)
    {
        isVirtualLoss = false;
    }

    std::vector<double> ucb(this->childNodes.size());
    if (isVirtualLoss)
    {
        for (int i = 0; i < this->childNodes.size(); ++i)
        {
            ucb[i] = this->childNodes[i]->score / (5 * this->childNodes[i]->num_thread_visited + 1) +
                     0.01 * std::sqrt(2.0 * std::log(this->visits) /
                                      (this->childNodes[i]->visits));
        }
    }
    else
    {
        for (int i = 0; i < this->childNodes.size(); ++i)
        {
            ucb[i] = this->childNodes[i]->score +
                     0.01 * std::sqrt(2.0 * std::log(this->visits) /
                                      (this->childNodes[i]->visits));
        }
    }

    double maxUCB = ucb[0];
    // if(maxUCB < 0)
    // {
    //     std::cout << "maxUCB: " << maxUCB << std::endl;
    // }
    std::vector<int> maxIndices(1, 0);
    for (int i = 1; i < ucb.size(); ++i)
    {
        if (ucb[i] > maxUCB)
        {
            maxUCB = ucb[i];
            maxIndices.clear();
            maxIndices.push_back(i);
        }
        else if (ucb[i] == maxUCB)
        {
            maxIndices.push_back(i);
        }
    }

    int selectedIndex = maxIndices[rand() % maxIndices.size()];
    // this->simDis += 1;
    this->num_thread_visited += 1;
    this->childNodes[selectedIndex]->num_thread_visited += 1;
    // if(this->childNodes[selectedIndex]->num_thread_visited > 1)
    // {
    //     std::cout << "childnode->num_thread_visited:" << this->childNodes[selectedIndex]->num_thread_visited << std::endl;
    //     std::cout << "childnode->score: " << this->childNodes[selectedIndex]->score << std::endl;
    // }

    return this->childNodes[selectedIndex];
}

/**
 * 根据UCB算法选择子节点, 注意num_thread_visited是用于构建虚拟loss的
 * @brief 选择子节点
 * @return TreeNode_ptr 选择的子节点
 */
TreeNode_ptr TreeNode::selection_singleThread()
{
    assert(!this->childNodes.empty() && "Child nodes should not be empty!");

    std::vector<double> ucb(this->childNodes.size());

    for (int i = 0; i < this->childNodes.size(); ++i)
    {
        ucb[i] = this->childNodes[i]->score +
                 1 * std::sqrt(2.0 * std::log(this->visits) /
                               (this->childNodes[i]->visits));
    }

    double maxUCB = ucb[0];
    // if(maxUCB < 0)
    // {
    //     std::cout << "maxUCB: " << maxUCB << std::endl;
    // }
    std::vector<int> maxIndices(1, 0);
    for (int i = 1; i < ucb.size(); ++i)
    {
        if (ucb[i] > maxUCB)
        {
            maxUCB = ucb[i];
            maxIndices.clear();
            maxIndices.push_back(i);
        }
        else if (ucb[i] == maxUCB)
        {
            maxIndices.push_back(i);
        }
    }

    int selectedIndex = maxIndices[rand() % maxIndices.size()];
    // this->simDis += 1;
    this->num_thread_visited += 1;
    this->childNodes[selectedIndex]->num_thread_visited += 1;
    // if(this->childNodes[selectedIndex]->num_thread_visited > 1)
    // {
    //     std::cout << "childnode->num_thread_visited:" << this->childNodes[selectedIndex]->num_thread_visited << std::endl;
    //     std::cout << "childnode->score: " << this->childNodes[selectedIndex]->score << std::endl;
    // }

    return this->childNodes[selectedIndex];
}

TreeNode_ptr TreeNode::findBestChild()
{
    assert(childNodes.size() != 0 && "Child nodes should not be empty!");

    std::vector<double> ucb(childNodes.size());
    for (int i = 0; i < childNodes.size(); ++i)
    {
        ucb[i] = childNodes[i]->score;
    }

    if (ucb.size() == 0)
    {
        std::cout << "childNodes.size(): " << childNodes.size() << std::endl;
        std::cout << "ucb.size() == 0" << std::endl;
        exit(0);
    }

    double maxUCB = ucb[0];
    std::vector<int> maxIndices(1, 0);
    for (int i = 1; i < ucb.size(); ++i)
    {
        if (ucb[i] > maxUCB)
        {
            maxUCB = ucb[i];
            maxIndices.clear();
            maxIndices.push_back(i);
        }
        else if (ucb[i] == maxUCB)
        {
            maxIndices.push_back(i);
        }
    }

    int selectedIndex = maxIndices[rand() % maxIndices.size()];
    return childNodes[selectedIndex];
}

std::string TreeNode::getParentKey()
{
    // 是否需要判断是否为空
    return this->hashKey.substr(0, this->hashKey.length() - 1);
}

/**
 * @brief  扩展节点, 后续这里是重点
 * @return TreeNode_ptr 选择的子节点
 */
void TreeNode::expansion(const Parameters &params, const std::vector<Eigen::Vector3f> &pathPnts)
{
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    check_candidateNodes = true;
    std::vector<TreeNode_ptr> allNodes;

    // 根据路径和机器人当前位置,确定机器人前进方向
    auto tmpState = this->rState;
    auto targetAngle = PLANNING::getTargetYawAndMoveDir(tmpState, pathPnts, params);
    // tmpState.pose.yaw = targetAngle.first;
    tmpState.moveDirection = targetAngle.second;

    float deltaYaw = tmpState.moveDirection - tmpState.pose.yaw;

    std::vector<MDT::RobotState> stateList;
    // FIXME: 如果旋转角度大于0.05,则只旋转
    if (fabs(deltaYaw) > params.onlyRotateThreshold)
    {
        stateList = PLANNING::getNextMCTSstateByExpert_rotate_moreState(tmpState, params);
    }
    else
    {
        tmpState.pose.yaw = targetAngle.first;
        stateList = PLANNING::getNextMCTSstateList_underConstrains_moreStates(tmpState, params);
    }

    // std::cout << "stateList.size: " << stateList.size() << std::endl;
    std::string key_element;

    if (stateList.size() > params.key_element.size())
    {
        std::cout << "stateList.size() >  params.key_element.size()" << std::endl;
        exit(0);
    }

    for (int i = 0; i < stateList.size(); ++i)
    {

        this->candidateNodes.push_back(stateList[i]);
    }

    for (int i = 0; i < candidateNodes.size(); ++i)
    {
        this->index_candidateN.push_back(i);
    }
}

/**
 * @brief  添加一个子节点
 * @return TreeNode_ptr 添加的子节点
 */
TreeNode_ptr TreeNode::addNode(int m, const Parameters &params)
{
    MDT::RobotState rState_ = this->candidateNodes[m];
    int hashStringIndex = this->index_candidateN[m];

    this->candidateNodes.erase(this->candidateNodes.begin() + m);
    this->index_candidateN.erase(this->index_candidateN.begin() + m);

    // 设置新节点的hashKey
    std::string key;
    key.append(this->hashKey);
    key.push_back(params.key_element[hashStringIndex]);

    std::shared_ptr<TreeNode> newNode_ptr = std::make_shared<TreeNode>(rState_, key);

    this->num_thread_visited += 1;
    newNode_ptr->num_thread_visited += 1;
    this->childNodes.push_back(newNode_ptr);
    return newNode_ptr;
}

void TreeNode::updateLocalNode(double score)
{
    this->visits += 1; // 新添加的节点首次加分, 在parallelMCTS中只有一次使用,就在仿真后
    this->score = score;
}

void TreeNode::updateLocalSimDis(double dis)
{
    this->simDis = dis;
}

double sigmoid(double x)
{
    return 1 / (1 + exp(-x));
}
double TreeNode::calculateLocalScore(double simDis, const Parameters &params)
{
    int nodeDepth = hashKey.size(); /////////// 有问题
    double score_ = 0.6 * pow(simDis / float(params.simStepNum), 1.0 / 3.0) + 0.3 * pow(rState.pose.x / 
        float(nodeDepth), 1.0 / 3.0) + 0.3 * pow(this->disToParents, 1.0 / 3.0);
    // std :: cout << "simSL:" << simDis / params.simStepNum ;
    // std :: cout << ", expandSL:" << rState.pose.x / float(nodeDepth);
    // std :: cout << ", disToParents:" << this->disToParents <<  std::endl;

    // double score_ = 0.0 * this->rState.pose.x + 3 * simDis / float(params.simStepNum) + 2* rState.pose.x / float(nodeDepth);//pow(, 3.0 / 3.0) ;
    // double score_ = sigmoid(simDis) + rState.pose.x / float(nodeDepth);//pow(, 3.0 / 3.0) ;
    // std :: cout << "sigmoid(simDis):" << sigmoid(simDis) ;
    // std :: cout << ", sigmoid(rState.pose.x / float(nodeDepth)):" << sigmoid(rState.pose.x / float(nodeDepth)) <<  std::endl;
    // std::cout << "score_:" << score_ << std::endl;
    // score_ = sigmoid(score_);
    return score_;
    //+ 0.1 * (this->rState.pose.x - this->parentNode->rState.pose.x)
}

#include <thread>
/**
 * @brief  模拟函数, 用于计算分值
 * @return double 分值
 */
double TreeNode::simulation(const MDT::RobotState &roState, const Parameters &params, const std::vector<Eigen::Vector3f> &pathPnts)
{
    // const int numThreads = 4;

    // // 存储线程对象和仿真距离结果
    // std::vector<std::thread> threads(numThreads);
    // std::vector<float> simDistances(numThreads);

    // // 创建线程并执行仿真函数
    // for (int i = 0; i < numThreads; ++i) {
    //     threads[i] = std::thread([&simDistances, i, roState, mapData]() {
    //         simDistances[i] = simulation_back(roState, mapData);
    //     });
    // }

    // // 等待所有线程完成
    // for (auto& thread : threads) {
    //     thread.join();
    // }

    // // // 输出仿真距离结果
    // // for (int i = 0; i < numThreads; ++i) {
    // //     std::cout << "Thread " << i << " simulation distance: " << simDistances[i] << std::endl;
    // // }

    // // 返回simDistances中的最大值
    // return *std::max_element(simDistances.begin(), simDistances.end());

    int count = 0;
    MDT::RobotState robotState_ = roState;
    while (count < params.simStepNum)
    {
        // 根据路径和机器人当前位置,确定机器人前进方向
        auto tmpState = robotState_;
        auto targetAngle = PLANNING::getTargetYawAndMoveDir(tmpState, pathPnts, params);
        // tmpState.pose.yaw = targetAngle.first;
        tmpState.moveDirection = targetAngle.second;
        float deltaYaw = tmpState.moveDirection - tmpState.pose.yaw;

        if (fabs(deltaYaw) > params.onlyRotateThreshold)
        {
            robotState_ = PLANNING::getNextMCTSstateRotate_forSim(tmpState, params);
        }
        else
        {
            tmpState.pose.yaw = targetAngle.first;
            robotState_ = PLANNING::getNextMCTSstate_forSim(tmpState, params);
        }

        // std::cout << "tmpState.moveDirection: " << tmpState.moveDirection << std::endl;
        // std::cout << "tmpState.pose.yaw: " << tmpState.pose.yaw << std::endl;
        count++;
    }

    auto simDis = PLANNING::getTwoStateDistanceAlongPath(roState, robotState_, pathPnts);

    // Eigen::Vector3f robotPos = {roState.pose.x, roState.pose.y, roState.pose.z};
    // auto startPntIndex = PLANNING::findTheNearestPathPointIndex(robotPos, pathPnts);

    // Eigen::Vector3f robotPos2 = {robotState_.pose.x, robotState_.pose.y, robotState_.pose.z};
    // auto endPntIndex = PLANNING::findTheNearestPathPointIndex(robotPos2, pathPnts);

    // //　计算仿真距离
    // float simDis = 0.0;
    // for(int i = startPntIndex; i <= endPntIndex; ++i)
    // {
    //     simDis += (pathPnts[i+1] - pathPnts[i]).norm();
    // }

    return simDis;
}

/**
 * @brief  回溯函数, 用于更新分值
 * @param  cnode 向上传播的节点
 * @return void
 */
int TreeNode::backpropagation(TreeNode_ptr cnode, const Parameters &params)
{
    int Flag = BP_TYPE::NO_BETTER_CHILD_BP;

    // std::string USE_POWER_MEAN = params.configMap.at("USE_POWER_MEAN");

    this->visits += 1;
    this->num_thread_visited -= 1;

    // 更新该儿子的分值,后续在SELECTION函数中会用到
    for (int i = 0; i < this->childNodes.size(); ++i)
    {

        if (this->childNodes[i]->hashKey == cnode->hashKey)
        {
            this->childNodes[i]->num_thread_visited -= 1;
            this->childNodes[i]->visits += 1;
            this->childNodes[i]->score = cnode->score;
        }
    }

    // 使用power mean
    if (params.USE_POWER_MEAN)
    {
        float p = std::stof(params.configMap.at("P_Parameter"));
        // 按照Joni的p公式,通过所有的child score计算自己的score
        for (int i = 0; i < this->childNodes.size(); ++i)
        {
            long double tmpScore = 0;
            if (this->childNodes[i]->score < 0)
            {
                continue;
            }
            tmpScore += std::pow(this->childNodes[i]->score, p) * float(this->childNodes[i]->visits) / float(this->visits); // 现在是均值
            this->score = std::pow(tmpScore, 1.0 / p);
        }

        // std::cout << "this->score : " << this->score << std::endl;

        // 防止因为计算精度问题导致的score异常
        if (this->score > 9999) // || this->score < 0.00001
        {
            std::cout << "\033[1;33m"; // 1表示粗体，33表示黄色
            std::cout << "power Mean Fail!";
            std::cout << "\033[0m"; // 恢复默认颜色
        }

        Flag = BP_TYPE::BETTER_CHILD_BP; // 强制反向传播
    }
    else // 使用Max BP
    {
        if (cnode->score > score)
        {
            score = cnode->score;
            Flag = BP_TYPE::BETTER_CHILD_BP;
        }
        if (!params.IsBestBP)
        {
            Flag = BP_TYPE::BETTER_CHILD_BP;
        }
    }

    // 处理负值反向传播部分.................................
    if (cnode->score < 0)
    {
        // 如果仍有候选节点,则不进行反向传播
        if (!this->candidateNodes.empty())
        {
            std::cout << "!this->candidateNodes.empty()" << std::endl;
            exit(0);
            return Flag;
        }

        // 如果备选儿子中有正值,则不进行反向传播,说明该节点仍有希望
        for (int i = 0; i < this->childNodes.size(); ++i)
        {
            if (this->childNodes[i]->score > 0)
            {
                // std::cout << "选儿子中有正值,则不进行反向传播,说明该节点仍有希望" << std::endl;
                return Flag;
            }
        }
        score = -999; // cnode->score;
        Flag = BP_TYPE::NEGATIVE_BP;
    }

    return Flag;
}

/**
 * @brief  回溯函数, 用于更新分值
 * @param  cnode 向上传播的节点
 * @return void
 */
int TreeNode::backpropagation_singleThread(TreeNode_ptr cnode, const Parameters &params)
{
    int Flag = BP_TYPE::NO_BETTER_CHILD_BP;

    this->visits += 1;
    this->num_thread_visited -= 1;

    // 更新该儿子的分值,后续在SELECTION函数中会用到
    for (int i = 0; i < this->childNodes.size(); ++i)
    {

        if (this->childNodes[i]->hashKey == cnode->hashKey)
        {
            this->childNodes[i]->num_thread_visited -= 1;
            this->childNodes[i]->visits += 1;
            this->childNodes[i]->score = cnode->score;
        }
    }

    if (cnode->score > score)
    {
        score = cnode->score;
    }

    // 处理负值反向传播部分.................................
    if (cnode->score < 0)
    {
        // 如果仍有候选节点,则不进行反向传播
        if (!this->candidateNodes.empty())
        {
            return Flag;
        }

        // 如果备选儿子中有正值,则不进行反向传播,说明该节点仍有希望
        for (int i = 0; i < this->childNodes.size(); ++i)
        {
            if (this->childNodes[i]->score > 0)
            {
                // std::cout << "选儿子中有正值,则不进行反向传播,说明该节点仍有希望" << std::endl;
                return Flag;
            }
        }
        score = -999; // cnode->score;
    }

    return Flag;
}

// 强制BP
void TreeNode::backpropagation_Force(TreeNode_ptr cnode)
{
    // score = cnode->score;
    this->visits += 1;
    this->num_thread_visited -= 1;

    // 更新该儿子的分值,后续在SELECTION函数中会用到
    for (int i = 0; i < this->childNodes.size(); ++i)
    {

        if (this->childNodes[i]->hashKey == cnode->hashKey)
        {
            this->childNodes[i]->num_thread_visited -= 1;
            this->childNodes[i]->visits += 1;
            this->childNodes[i]->score = cnode->score;
        }
    }
}

// 如果某节点扩展时无候选节点,那么反向传播到这里,更新该节点在其父亲中保存的值.
void TreeNode::updateChildBP(TreeNode_ptr cnode)
{

    // 如果子节点扩展时就无候选节点,那么反向传播到这里,visits不会加1, 因为后边该节点会继续SELECTION,再次反向传播时仍会+1.
    // 然而num_thread_visited会-1,因为后边该节点在SELECTION环节num_thread_visited会+1,所以这里要-1.

    // this->visits += 1;
    this->num_thread_visited -= 1;
    // 更新该儿子的分值,后续在SELECTION函数中会用到
    for (int i = 0; i < this->childNodes.size(); ++i)
    {

        if (this->childNodes[i]->hashKey == cnode->hashKey)
        {
            this->childNodes[i]->num_thread_visited -= 1;
            this->childNodes[i]->visits += 1;
            this->childNodes[i]->score = -8888; // 为什么这里的值会显示在hash tree 里? 这里只是更新父节点保存的儿子信息,方便SELECTION函数使用.
        }
    }
}
