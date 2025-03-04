#ifndef _SEARCH_TREE_H
#define _SEARCH_TREE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <mutex>
#include <myDataType.h>
#include <planning.h>
#include <memory>
#include <userParameter.h>

enum BP_TYPE {
    BETTER_CHILD_BP = 0,
    NEGATIVE_BP = 1,
    NO_BETTER_CHILD_BP = 2
};


class TreeNode  : public std::enable_shared_from_this<TreeNode> {
    public:
        MDT::RobotState rState;
        double score;
        std::vector<MDT::RobotState> candidateNodes;
        std::vector<std::shared_ptr<TreeNode>> childNodes;
        // std::shared_ptr<TreeNode> parentNode;
        std::vector<int> index_candidateN;  // 用于节点的删减的索引
        std::string hashKey;
        std::vector<std::string> val; // 字符串, 可重复利用不用放在这里
        int visits;
        int num_thread_visited;
        bool check_candidateNodes;
        float simDis;
        float disToParents;


        TreeNode(const MDT::RobotState& rState_, const std::string& hashKey_)
            : rState(rState_), score(0.0), visits(0), hashKey(hashKey_),
            num_thread_visited(0), simDis(0.0), check_candidateNodes(false) {
            // Initialize other members if needed
        }
        std::shared_ptr<TreeNode> selection(bool isVirtualLoss);
        std::shared_ptr<TreeNode> selection_singleThread();
        void expansion(const Parameters &params, const std::vector<Eigen::Vector3f>& pathPnts);
        std::shared_ptr<TreeNode> addNode(int m, const Parameters &params);
        void updateLocalNode(double score);
        double calculateLocalScore(double simDis, const Parameters &params);
        double simulation(const MDT::RobotState& roState, const Parameters &params, const std::vector<Eigen::Vector3f>& pathPnts);
        std::shared_ptr<TreeNode> findBestChild();
        void updateLocalSimDis(double dis);
        std::string getParentKey();
        int backpropagation(std::shared_ptr<TreeNode> cnode, const Parameters &params);
        int backpropagation_singleThread(std::shared_ptr<TreeNode> cnode, const Parameters &params);
        void updateChildBP(std::shared_ptr<TreeNode> cnode);
        void backpropagation_Force(std::shared_ptr<TreeNode> cnode);
};


typedef std::shared_ptr<TreeNode> TreeNode_ptr;









#endif




