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

GridPolyLine IntersectBorder::getIntersectBorder(const GridPt &start, const GridPt &end)
{
    GridPolyLine path;
    GridPt start_idx = start, goal_idx = goal;
    GridPt idx, start_border_idx, tmp_idx, revisit_idx;
    idx = start_idx;

    // state pointer: -1 for outside, n for inside the n-th polygon
    int in_poly_ptr_ = 0;
    // state pointer: -1 for outside, n for inside the n-th corridor
    int in_corridor_ptr_ = 0;

    if (border_check_.inPoly(idx, in_corridor_ptr_) == -1)
    {
        std::cerr << "Start point not in poly0!" << std::endl;
        return path;
    }

    // Find start border (x direction)
    // FIXME: This may find a start point in the middle of the corridor
    while (border_check_.inBorder(idx, in_corridor_ptr_) != -1)
    {
        idx[0]++;
    }
    idx[0]--; // Back to last inBorder
    start_border_idx = idx;
    path.push_back(start_border_idx);


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
            tmp_idx = idx + c8_cw.at(i);
            // flag: pointer out of border
            if (!out_corridor_flag && !inBorderJudge(Corridor, map_, tmp_idx))
            {
                out_corridor_flag = true;
            }
            // flag: pointer back from border
            if (out_corridor_flag && inBorderJudge(Corridor, map_, tmp_idx))
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
                    break;
                }
            }
        }
        if (revisit_flag)
        {
            printf("Warning: Revisit(%d %d)", revisit_idx[0], revisit_idx[1]);
            path.push_back(revisit_idx);
            idx = revisit_idx;
        }

    } while (idx[0] != start_border_idx[0] || idx[1] != start_border_idx[1]);

    return path;
}