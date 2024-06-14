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

#include "legged_traj_search/poly_traj/intersect_border.hpp"

IntersectBorder::IntersectBorder(PolyCorridor &poly_corridor, BorderCheck &border_check)
    : poly_corridor_(poly_corridor), border_check_(border_check)
{
}

bool IntersectBorder::checkPointProjectInPoly(const Point &pt, int poly_idx)
{
    GridPt grid_pt = border_check_.getIndexRemap().pos2Grid(pt);
    return border_check_.inPoly(grid_pt, poly_idx);
}

bool IntersectBorder::checkPointProjectInPoly(const GridPt &pt, int poly_idx)
{
    return border_check_.inPoly(pt, poly_idx);
}

bool IntersectBorder::getIntersectBorder(const Point &start, const Point &goal,
                                         GridPolyLine &border, int max_iter)
{
    GridPt start_grid = border_check_.getIndexRemap().pos2Grid(start);
    GridPt goal_grid = border_check_.getIndexRemap().pos2Grid(goal);

    return getIntersectBorder(start_grid, goal_grid, border, max_iter);
}

bool IntersectBorder::getIntersectBorder(const GridPt &start, const GridPt &goal,
                                         GridPolyLine &border, int max_iter)
{
    border.clear();
    // TODO: whether to remove revisited path? (it can alsh be handled in VisGraph)
    GridPolyLine &tmp_border = border;
    std::vector<GridPt> turning_points;

    GridPt idx, start_border_idx, tmp_idx, revisit_idx;
    idx = start;
    if (!checkPointProjectInPoly(start, 0))
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
    tmp_border.emplace_back(start_border_idx);
    GridPointer grid_ptr(start_border_idx);

    // Find the intersect border
    // FIXME: Sometimes it stucks (loop)
    int cnt = 0;
    int tmp_incorridor_idx = -1;
    do
    {
        cnt++;
        if (cnt > max_iter)
        {
            std::cerr << "getCorriderIntersectBorder() stucks in loop!" << std::endl;
            printf("start(%d %d), now(%d %d)\n",
                   start_border_idx[0], start_border_idx[1],
                   grid_ptr.getState()[0], grid_ptr.getState()[1]);
            return false;
        }

        bool out_corridor_flag = false;
        bool revisit_flag = false;
        bool nosol_flag = true;
        for (int i = 0; i < 9; i++)
        {
            tmp_idx = grid_ptr.getNextCandidateState();
            tmp_incorridor_idx = border_check_.inBorder(tmp_idx);
            // flag: pointer out of border
            if (!out_corridor_flag && tmp_incorridor_idx == -1)
            {
                out_corridor_flag = true;
            }
            // flag: pointer back from border
            if (out_corridor_flag && tmp_incorridor_idx != -1)
            {
                nosol_flag = false;
                if (tmp_border.size() > 1 && tmp_idx.isApprox(tmp_border.at(tmp_border.size() - 2)))
                {
                    revisit_flag = true;
                    revisit_idx = tmp_idx;
                }
                else
                {
                    revisit_flag = false;
                    if ((tmp_idx - grid_ptr.getState()).isApprox(grid_ptr.getLastMove()))
                        tmp_border.back() = tmp_idx;
                    else
                        tmp_border.emplace_back(tmp_idx);
                    grid_ptr.updateState(tmp_idx);
                    break;
                }
            }
        }
        if (revisit_flag)
        {
            std::cerr << "Warning: Revisit(" << revisit_idx[0] << " " << revisit_idx[1] << ")" << std::endl;
            turning_points.emplace_back(grid_ptr.getState());
            tmp_border.emplace_back(revisit_idx);
            grid_ptr.updateState(revisit_idx);
        }

        if (nosol_flag)
        {
            std::cerr << "Warning: No next border point found at cnt=" << cnt << std::endl;
            return false;
        }
    } while (!grid_ptr.getState().isApprox(start_border_idx));
    // IMPORTANT: the last point should NOT be the same as the first point
    tmp_border.pop_back();

    // Remove revisited path
    // for (int i = 0; i < tmp_border.size(); i++)
    // {
    //     border.emplace_back(tmp_border.at(i));
    //     if (turning_points.size() > 0 && tmp_border.at(i).isApprox(turning_points.front()))
    //     {
    //         turning_points.erase(turning_points.begin());
    //         int cnt = 1;
    //         while (tmp_border.at((i + cnt) % tmp_border.size())
    //                    .isApprox(tmp_border.at((i - 1 + tmp_border.size()) % tmp_border.size())))
    //         {
    //             if (border.size() > 0)
    //                 border.pop_back(); // FIXME: if it is empty?
    //             cnt++;
    //         }
    //         i = (i + cnt - 1) % tmp_border.size();
    //     }
    // }

    // TODO: Straight line combine for speed

    return true;
}