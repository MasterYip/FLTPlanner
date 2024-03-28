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
 * @note Border is defined `clockwise`
 * @param b1 Border segment start point
 * @param b2 Border segment end point
 * @param p1 Path segment start point
 * @param p2 Path segment end point
 * @return -1 Uncertain, segments do not intersect
 * @return 0 Path segment is outward
 * @return 1 Path segment is inward
 */
int isInward(const Point &b1, const Point &b2,
             const Point &p1, const Point &p2)
{
    if (segmentIntersect(b1, b2, p1, p2) == IntersectType::None)
    {
        return -1;
    }
    else
    {
        if (crossProd(b2 - b1, p2 - p1) < 0)
            return 1;
        else
            return 0;
    }
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
    }
}