/**
 * @file intersect_border.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "gcs_traj_opt/poly_traj/intersect_border.hpp"

IntersectBorder::IntersectBorder(PolyCorridor &poly_corridor, BorderCheck &border_check)
    : poly_corridor_(poly_corridor), border_check_(border_check)
{
}

[[deprecated]] void IntersectBorder::updatePtr(const GridPt &grid2d)
{
    // int tmp = border_check_.inPoly(grid2d, corridor_ptr_);
    int tmp = border_check_.inPoly(grid2d);

    if (tmp != -1 && tmp != poly_ptr_)
    {
        if (tmp - poly_ptr_ == 1)
        {
            corridor_ptr_ = tmp;
            if (corridor_ptr_ > (int)poly_corridor_.getCorridorSize() - 1)
                corridor_ptr_ = poly_corridor_.getCorridorSize() - 1;
        }
        else if (tmp - poly_ptr_ == -1)
        {
            corridor_ptr_ = tmp - 1;
            if (corridor_ptr_ < 0)
                corridor_ptr_ = 0;
        }
        poly_ptr_ = tmp;
    }
}

GridPolyLine IntersectBorder::getIntersectBorder(const GridPt &start, const GridPt &goal)
{
    GridPolyLine path;
    GridPt start_idx = start, goal_idx = goal;
    GridPt idx, start_border_idx, tmp_idx, revisit_idx;
    // TODO: put idx in the class
    idx = start_idx;

    // BUG: this is not a good way to check if the start point is in the polyhedra
    resetPtr();
    if (border_check_.inPoly(idx) != 0)
    {
        std::cerr << "Start point not in poly 0! find in poly " << border_check_.inPoly(idx) << std::endl;
        return path;
    }

    // Find start border (x direction)
    // FIXME: This may find a start point in the middle of the corridor
    while (border_check_.inBorder(idx) != -1)
    {
        idx[0]++;
        // updatePtr(idx);
    }
    idx[0]--; // Back to last inBorder
    start_border_idx = idx;
    path.push_back(start_border_idx);
    // updatePtr(idx);

    // Find the intersect border
    // FIXME: Sometimes it stucks (loop)
    uint max_tries = 200;
    uint cnt = 0;
    do
    {
        cnt++;
        if (cnt > max_tries)
        {
            std::cerr << "getCorriderIntersectBorder() stucks!" << std::endl;
            printf("start(%d %d), now(%d %d)\n", start_border_idx[0], start_border_idx[1], idx[0], idx[1]);
            break;
        }

        bool out_corridor_flag = false;
        bool revisit_flag = false;
        for (int i = 0; i < 9; i++)
        {
            // PROBLEM: What will happen if we do not update for tmp_idx?
            tmp_idx = idx + c8_cw.at(i);
            // flag: pointer out of border
            if (!out_corridor_flag && border_check_.inBorder(tmp_idx) == -1)
            {
                out_corridor_flag = true;
            }
            // flag: pointer back from border
            if (out_corridor_flag && border_check_.inBorder(tmp_idx) != -1)
            {

                if (path.size() > 1 && tmp_idx[0] == path.at(path.size() - 2)[0] && tmp_idx[1] == path.at(path.size() - 2)[1])
                {
                    revisit_flag = true;
                    revisit_idx = tmp_idx;
                }
                else
                {
                    revisit_flag = false;
                    path.push_back(tmp_idx);
                    idx = tmp_idx;
                    // updatePtr(idx);
                    break;
                }
            }
        }
        if (revisit_flag)
        {
            printf("Warning: Revisit(%d %d)", revisit_idx[0], revisit_idx[1]);
            path.push_back(revisit_idx);
            idx = revisit_idx;
            // updatePtr(idx);
        }

    } while (idx[0] != start_border_idx[0] || idx[1] != start_border_idx[1]);

    return path;
}