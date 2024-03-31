#ifndef _SAVE_HASH_TABLE_H
#define _SAVE_HASH_TABLE_H

#include <user.h>
#include <zobrist_hash.h>
#include <search_tree.h>
#include <hit_spider/hexapod_State.h>



namespace SAVE_HASH_TABLE
{
    struct saveData
    {
        MDT::RobotState rState;
        double score;
        double simDis;
        std::string hashKey;
        std::string parentKey;
        int visits;
        int num_thread_visited;

        // saveData(const TreeNode &node)
        //     : rState(node.rState), score(node.score), hashKey(node.hashKey), parentKey(node.parentNode->hashKey),
        //     visits(node.visits), num_thread_visited(node.num_thread_visited) {}
        saveData(TreeNode &node)
        {
            rState = node.rState;
            score = node.score;
            simDis = node.simDis;
            hashKey = node.hashKey;
            if (node.getParentKey().empty())
                parentKey = "null";
            else
                parentKey = node.getParentKey();
            visits = node.visits;
            num_thread_visited = node.num_thread_visited;
        }

        saveData(){}
    };
    

    void saveBinaryFile(const std::vector<SAVE_HASH_TABLE::saveData> &keyInfo, const std::string& filename);
    std::vector<SAVE_HASH_TABLE::saveData> readBinaryFile(const std::string& filename);

}


#endif