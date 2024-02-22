/**
 * @file border_check.hpp
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
#include <grid_map_core/grid_map_core.hpp>
/* internal project header files */
#include "gcs_traj_opt/geo_utils/polycorridor.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils_2d;

class BorderCheck
{
private:
    /* data */
    std::string map_layer_;
    // PROBLEM: How to share the map_ground_ and map_ceiling_ with the other class?
    const grid_map::GridMap &map_ground_;
    const grid_map::GridMap &map_ceiling_;
    PolyCorridor poly_corridor_;

public:
    BorderCheck(const PolyCorridor &poly_corridor,
                const grid_map::GridMap &map_ground,
                const grid_map::GridMap &map_ceiling,
                const std::string map_layer = "elevation");

    /**
     * @brief Check if the given position is in the corridor intersection border
     *
     * @param pos2d Position in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int inBorder(const Eigen::Vector2d &pos2d, uint corridor_idx);
    int inBorder(const GridPt &grid2d, uint corridor_idx);
};