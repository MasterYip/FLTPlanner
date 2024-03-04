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

bool IntersectBorder::getIntersectBorder(const Point &start, const Point &goal, GridPolyLine &border)
{
    GridPt start_idx, goal_idx;
    border_check_.getMap().getIndex(start, start_idx);
    border_check_.getMap().getIndex(goal, goal_idx);
    return getIntersectBorder(start_idx, goal_idx, border);
}

bool IntersectBorder::getIntersectBorder(const GridPt &start, const GridPt &goal, GridPolyLine &border)
{
    border.clear();
    GridPt start_idx = start;
    // GridPt goal_idx = goal;
    GridPt idx, start_border_idx, tmp_idx, revisit_idx;
    idx = start_idx;

    // BUG: this is not a good way to check if the start point is in the polyhedra
    if (border_check_.inPoly(idx) != 0)
    {
        std::cerr << "Start point not in poly 0! find in poly " << border_check_.inPoly(idx) << std::endl;
        return false;
    }

    // Find start border (x direction)
    // FIXME: This may find a start point in the middle of the corridor
    while (border_check_.inBorder(idx) != -1)
    {
        idx[0]++;
    }
    idx[0]--; // Back to last inBorder
    start_border_idx = idx;
    border.push_back(start_border_idx);

    // Find the intersect border
    // FIXME: Sometimes it stucks (loop)
    uint max_tries = 200;
    uint cnt = 0;
    // int idx_incorridor_idx1 = border_check_.inBorder(idx);
    // int idx_incorridor_idx2 = border_check_.inCorridor(idx, idx_incorridor_idx1 + 1);
    int tmp_incorridor_idx = -1;
    do
    {
        cnt++;
        if (cnt > max_tries)
        {
            std::cerr << "getCorriderIntersectBorder() stucks in loop!" << std::endl;
            printf("start(%d %d), now(%d %d)\n", start_border_idx[0], start_border_idx[1], idx[0], idx[1]);
            return true; // FIXME: to check algo stablity
        }

        bool out_corridor_flag = false;
        bool revisit_flag = false;
        bool nosol_flag = true;
        for (int i = 0; i < 9; i++)
        {
            tmp_idx = idx + c8_cw.at(i);
            tmp_incorridor_idx = border_check_.inBorder(tmp_idx);
            // flag: pointer out of border
            // if (!out_corridor_flag && (tmp_incorridor_idx == -1 ||
            //                            (tmp_incorridor_idx != idx_incorridor_idx1 &&
            //                             tmp_incorridor_idx != idx_incorridor_idx2)))
            if (!out_corridor_flag && tmp_incorridor_idx == -1)
            {
                out_corridor_flag = true;
            }
            // flag: pointer back from border
            // if (out_corridor_flag && tmp_incorridor_idx != -1 &&
            //     (tmp_incorridor_idx == idx_incorridor_idx1 ||
            //      tmp_incorridor_idx == idx_incorridor_idx2))
            if (out_corridor_flag && tmp_incorridor_idx != -1)
            {
                nosol_flag = false;
                if (border.size() > 1 && tmp_idx.isApprox(border.at(border.size() - 2)))
                {
                    revisit_flag = true;
                    revisit_idx = tmp_idx;
                }
                else
                {
                    revisit_flag = false;
                    border.push_back(tmp_idx);
                    idx = tmp_idx;
                    // idx_incorridor_idx1 = tmp_incorridor_idx;
                    // idx_incorridor_idx2 = border_check_.inCorridor(idx, idx_incorridor_idx1 + 1);
                    break;
                }
            }
        }
        if (revisit_flag)
        {
            printf("Warning: Revisit(%d %d)", revisit_idx[0], revisit_idx[1]);
            border.push_back(revisit_idx);
            idx = revisit_idx;
            // idx_incorridor_idx1 = tmp_incorridor_idx;
            // idx_incorridor_idx2 = border_check_.inCorridor(idx, idx_incorridor_idx1 + 1);
        }

        if (nosol_flag)
        {
            std::cerr << "Warning: No next border point found!" << std::endl;
            return true;
        }

    } while (idx[0] != start_border_idx[0] || idx[1] != start_border_idx[1]);
    // IMPORTANT: the last point should NOT be the same as the first point
    border.pop_back();
    return true;
}