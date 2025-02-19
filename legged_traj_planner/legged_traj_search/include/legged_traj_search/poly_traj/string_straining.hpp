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
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils_2d;

/**
 * @brief Given the border segment & path segment, check if the path segment is inward border segment
 * @note Border is defined `clockwise`, and make sure the 2 segments intersect
 * @param b1 Border segment start point
 * @param b2 Border segment end point
 * @param p1 Path segment start point
 * @param p2 Path segment end point
 * @return false Path segment is outward
 * @return true Path segment is inward
 */
bool inline isInward(const GridPt &b1, const GridPt &b2,
                     const GridPt &p1, const GridPt &p2)
{
    return crossProd(GridPt(b2 - b1), GridPt(p2 - p1)) <= 0;
}

bool inline isInward(const GridPt &b1, const GridPt &b2, const GridPt &b3,
                     const GridPt &p1, const GridPt &p2)
{
    return isConcavePoint(b2, b1, b3, true) ? (isInward(b1, b2, p1, p2) || isInward(b2, b3, p1, p2)) : (isInward(b1, b2, p1, p2) && isInward(b2, b3, p1, p2));
}

// TODO: Try fix this
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
    // FIXME: This can't solve complex problem
    bool search(GridPolyLine &result, uint max_lap = 8)
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
                if (k == IntersectType::Middle)
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
                        if (isInward(b1m, b1, b2, p1, p2) != isInward(b1m, b1, b2, p2, p1))
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
                        if (isInward(b1, b2, p2, p1) != isInward(b1, b2, p2, p2p))

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
                        if (isInward(b1m, b1, b2, p2, p1) != isInward(b1m, b1, b2, p2, p2p))
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
                    result = grid_traj_;
                    return false;
                }
            }
        }
        result = grid_traj_;
        return true;
    }
};