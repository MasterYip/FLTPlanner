/**
 * @file gcs_astar_search.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_search/astar/stlastar.h"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"
#include "legged_traj_search/geo_utils/visibility.hpp"
using namespace geo_utils_2d;

class GridPtState : public AStarState<GridPtState>
{
private:
    uint pt_idx_;
    VisibilityGraph *vis_graph_;

public:
    GridPtState() : pt_idx_(0), vis_graph_(nullptr){};
    GridPtState(uint pt_idx, VisibilityGraph *vis_graph) : pt_idx_(pt_idx), vis_graph_(vis_graph){};
    uint getPtIdx()
    {
        return pt_idx_;
    };
    float GoalDistanceEstimate(GridPtState &nodeGoal) override
    {
        return (vis_graph_->getPt(pt_idx_) - vis_graph_->getPt(nodeGoal.getPtIdx())).matrix().norm();
    }; // Heuristic function which computes the estimated cost to the goal node
    bool IsGoal(GridPtState &nodeGoal) override
    {
        return nodeGoal.getPtIdx() == pt_idx_;
    }; // Returns true if this node is the goal node
    bool GetSuccessors(AStarSearch<GridPtState> *astarsearch, GridPtState *parent_node) override
    {
        GridPtState newnode;
        for (uint i = 0; i < vis_graph_->size(); i++)
        {
            if ((parent_node && i != parent_node->getPtIdx() && i != pt_idx_ && vis_graph_->isVisibile(pt_idx_, i)) ||
                (!parent_node && i != pt_idx_ && vis_graph_->isVisibile(pt_idx_, i)))
            {
                newnode = GridPtState(i, vis_graph_);
                astarsearch->AddSuccessor(newnode);
            }
        }
        return true;
    }; // Retrieves all successors to this node and adds them via astarsearch.addSuccessor()
    float GetCost(GridPtState &successor) override
    {
        return (vis_graph_->getPt(pt_idx_) - vis_graph_->getPt(successor.getPtIdx())).matrix().norm();
    }; // Computes the cost of travelling from this node to the successor node
    bool IsSameState(GridPtState &rhs) override
    {
        return rhs.getPtIdx() == pt_idx_;
    }; // Returns true if this node is the same as the rhs node
    size_t Hash()
    {
        return pt_idx_;
    }; // Returns a hash for the state
};

inline bool GCS_AStarSearch(VisibilityGraph &vis_graph, std::vector<GridPt> &path)
{
    // A* Search
    AStarSearch<GridPtState> astarsearch;

    GridPtState start_state(0, &vis_graph);
    GridPtState goal_state(1, &vis_graph);
    astarsearch.SetStartAndGoalStates(start_state, goal_state);
    uint SearchState;
    uint SearchSteps = 0;
    do
    {
        SearchState = astarsearch.SearchStep();
        SearchSteps++;
    } while (SearchState == AStarSearch<GridPtState>::SEARCH_STATE_SEARCHING);
    if (SearchState == AStarSearch<GridPtState>::SEARCH_STATE_SUCCEEDED)
    {
        GridPtState *node = astarsearch.GetSolutionStart();
        path.clear();
        path.emplace_back(vis_graph.getPt(node->getPtIdx()));
        while (true)
        {
            node = astarsearch.GetSolutionNext();
            if (!node)
                break;
            path.emplace_back(vis_graph.getPt(node->getPtIdx()));
        };
        astarsearch.FreeSolutionNodes();
        astarsearch.EnsureMemoryFreed();
        return true;
    }
    astarsearch.FreeSolutionNodes();
    // astarsearch.EnsureMemoryFreed();
    return false;
}