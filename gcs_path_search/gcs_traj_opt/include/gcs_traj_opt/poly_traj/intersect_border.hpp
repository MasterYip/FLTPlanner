/**
 * @file intersect_border.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-11
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
#include "gcs_traj_opt/geo_utils/polycorridor.hpp"
#include "gcs_traj_opt/poly_traj/border_check.hpp"

using namespace geo_utils_2d;

class GridPointer
{
private:
    GridPt state_;
    GridPt last_state_;
    GridPt last_move_;
    uint candidate_ptr_ = 0;

    // BUG
    // TODO: the direction should decide by the previous direction
    // Connectivity 8 Clockwise Search
    // 7 8 1
    // 6 * 2
    // 5 4 3
    std::vector<GridPt> c8_cw = {{1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}};

public:
    GridPointer(const GridPt &state)
        : state_(state), last_state_(state)
    {
    }

    GridPt getNextCandidateState()
    {
        GridPt candidate = state_ + c8_cw[candidate_ptr_];
        candidate_ptr_ = (candidate_ptr_ + 1) % c8_cw.size();
        return candidate;
    }

    // Only delta in c8_cw is allowed
    void moveState(const GridPt &delta)
    {
        if (delta.isApprox(GridPt::Zero()))
            return;
        else if (delta[0] < -1 || delta[0] > 1 || delta[1] < -1 || delta[1] > 1)
            throw std::runtime_error("Invalid delta: " + std::to_string(delta[0]) + ", " + std::to_string(delta[1]));

        last_state_ = state_;
        state_ += delta;
        last_move_ = delta;

        for (uint i = 0; i < c8_cw.size(); i++)
        {
            if (c8_cw[i].isApprox(-last_move_))
            {
                candidate_ptr_ = (i + 1) % c8_cw.size();
                break;
            }
        }
    }

    void moveState(int dx, int dy)
    {
        moveState(GridPt(dx, dy));
    }

    void updateState(const GridPt &state)
    {
        GridPt delta = state - state_;
        moveState(delta);
    }

    GridPt getState() const
    {
        return state_;
    }
};

class IntersectBorder
{
private:
    PolyCorridor &poly_corridor_;
    BorderCheck &border_check_;

public:
    IntersectBorder(PolyCorridor &poly_corridor, BorderCheck &border_check);

    /**
     * @brief Check the given point projection on aux plain (T) is in the polyhedra (used for start & goal)
     * 
     * @param pt 
     * @param poly_idx 
     * @return true 
     * @return false 
     */
    bool checkPointProjectInPoly(const GridPt &pt, int poly_idx);

    bool checkPointProjectInPoly(const Point &pt, int poly_idx);

    bool getIntersectBorder(const GridPt &start, const GridPt &goal, GridPolyLine &border, int max_iter = 2000);

    bool getIntersectBorder(const Point &start, const Point &goal, GridPolyLine &border, int max_iter = 2000);

    PolyCorridor &getPolyCorridor()
    {
        return poly_corridor_;
    }

    BorderCheck &getBorderCheck()
    {
        return border_check_;
    }
};
