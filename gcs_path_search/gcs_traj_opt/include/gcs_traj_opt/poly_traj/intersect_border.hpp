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
#include "gcs_traj_opt/geo_utils/border_check.hpp"

using namespace geo_utils_2d;

class IntersectBorder
{
private:
    PolyCorridor &poly_corridor_;
    BorderCheck &border_check_;

    // Connectivity 8 Clockwise Search
    // 7 8 1
    // 6 * 2
    // 5 4 3
    std::vector<GridPt> c8_cw = {{1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}};

public:
    IntersectBorder(PolyCorridor &poly_corridor,
                    BorderCheck &border_check) : poly_corridor_(poly_corridor),
                                                 border_check_(border_check){};
    // GridPolyLine getIntersectBorder(const Point &start, const Point &end);
    GridPolyLine getIntersectBorder(const GridPt &start, const GridPt &end);
}
