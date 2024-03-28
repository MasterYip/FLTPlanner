/**
 * @file string_straining.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-28
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
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils_2d;

/**
 * @brief Given the border segment & path segment, check if the path segment is inward border segment
 * @note Border is defined `clockwise`, and make sure the 2 segments intersect
 * @param b1 Border segment start point
 * @param b2 Border segment end point
 * @param p1 Path segment start point
 * @param p2 Path segment end point
 * @return -1 Uncertain, segments do not intersect (unused)
 * @return 0 Path segment is outward
 * @return 1 Path segment is inward
 */
int isInward(const Point &b1, const Point &b2,
             const Point &p1, const Point &p2)
{
    // if (segmentIntersect(b1, b2, p1, p2) == IntersectType::None)
    // {
    //     return -1;
    // }
    // else
    // {
    if (crossProd(b2 - b1, p2 - p1) < 0)
        return 1;
    else
        return 0;
    // }
}

class StringStrainingSearch
{
private:
    GridPolyLine border_;
    GridPolyLine grid_traj_;
    GridPt start_;
    GridPt goal_;

public:
    StringStrainingSearch(const GridPolyLine &border, const GridPt &start, const GridPt &goal)
        : border_(border), start_(start), goal_(goal)
    {
        grid_traj_.emplace_back(start);
        grid_traj_.emplace_back(goal);
    }

    bool search(GridPolyLine &result, uint max_lap = 4)
    {
        GridPt b1, b2, b3;
        GridPt p1, p2, p3;
        size_t i = 0;
        size_t no_update_cnt = 0;
        uint lap_cnt = 0;
        while (no_update_cnt < border_.size() + 1)
        {
            b1 = border_.at(i);
            b2 = border_.at((i + 1) % border_.size());
            for (size_t j = 0; j < grid_traj_.size() - 1; j++)
            {
                p1 = grid_traj_.at(j);
                p2 = grid_traj_.at(j + 1);
                IntersectType k = segmentIntersect(b1, b2, p1, p2);

                if (k == IntersectType::Middle && isInward(b1, b2, p1, p2) == 1)
                {
                    grid_traj_.insert(grid_traj_.begin() + j + 1, b2);
                    no_update_cnt = 0;
                    break;
                }
                else if (k == IntersectType::EndMid)
                {

                    GridPt b1m = border_.at((i - 1 + border_.size()) % border_.size());
                    IntersectType k2 = segmentIntersect(b1m, b1, p1, p2);
                    if (k2 == IntersectType::EndMid)
                    {
                        bool concave = isConcavePoint(b1, b1m, b2, true);
                        if (concave && (isInward(b1, b2, p1, p2) == 1 || isInward(b1m, b1, p1, p2) == 1) ||
                            !concave && isInward(b1, b2, p1, p2) == 1 && isInward(b1m, b1, p1, p2) == 1)
                        {
                            grid_traj_.insert(grid_traj_.begin() + j + 1, b2);
                            no_update_cnt = 0;
                            break;
                        }
                    }
                }
                else if (k == IntersectType::MidEnd)
                {
                    if (j < grid_traj_.size() - 2)
                    {
                        GridPt p2p = grid_traj_.at(j + 2);
                        if (isInward(b1, b2, p1, p2) == 1 && isInward(b1, b2, p2, p2p) == 1)
                        {
                            grid_traj_.at(j + 1) = b2;
                            no_update_cnt = 0;
                            break;
                        }
                    }
                }
                else if (k == IntersectType::End)
                {
                    if (b1.isApprox(p2) && j < grid_traj_.size() - 2) // This will trigger before b1 = p1
                    {
                        GridPt b1m = border_.at((i - 1 + border_.size()) % border_.size());
                        GridPt p2p = grid_traj_.at(j + 2);
                        bool concave = isConcavePoint(b1, b1m, b2, true);
                        if ((concave && (isInward(b1, b2, p1, p2) == 1 || isInward(b1m, b1, p1, p2) == 1) &&
                             (isInward(b1, b2, p2, p2p) == 1 || isInward(b1m, b1, p2, p2p) == 1)) ||
                            (!concave && isInward(b1, b2, p1, p2) == 1 && isInward(b1m, b1, p1, p2) == 1 &&
                             isInward(b1, b2, p2, p2p) == 1 && isInward(b1m, b1, p2, p2p) == 1))
                        {
                            grid_traj_.at(j + 1) = b2;
                            no_update_cnt = 0;
                            break;
                        }
                    }
                }
            }
            no_update_cnt++;
            i = (i + 1) % border_.size();
            if (i == 0)
            {
                lap_cnt++;
                if (lap_cnt >= max_lap)
                {
                    return false
                }
            }
        }
        result = grid_traj_;
        return true;
    }
}