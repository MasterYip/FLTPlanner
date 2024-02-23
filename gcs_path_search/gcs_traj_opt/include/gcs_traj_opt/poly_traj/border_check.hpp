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
    std::string ground_layer_;
    std::string ceiling_layer_;
    bool enable_ceiling_;
    // PROBLEM: Is this safe to use reference here?
    PolyCorridor &poly_corridor_;
    // PROBLEM: How to share the map_ground_ and map_ceiling_ with the other class?
    const grid_map::GridMap &map_;

public:
    BorderCheck(PolyCorridor &poly_corridor,
                const grid_map::GridMap &map,
                const std::string ground_layer = "elevation",
                const bool enable_ceiling = false,
                const std::string ceiling_layer = "ceiling");

    double queryHeight(const Eigen::Vector2d &pos2d, uint corridor_idx);

    double queryHeight(const GridPt &grid2d, uint corridor_idx);

    /**
     * @brief Check if the given position is in the corridor intersection border
     *
     * @param pos2d Position in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int inBorder(const Eigen::Vector2d &pos2d, uint corridor_idx);

    /**
     * @brief Check if the given position is in the corridor intersection border
     *
     * @param grid2d GridPoint in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int inBorder(const GridPt &grid2d, uint corridor_idx);

    /**
     * @brief Check if the given position is in the polyhedra
     *
     * @param grid2d GridPoint in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the polyhedra that the position is in, -1 if not in any segment
     */
    int inPoly(const GridPt &grid2d, uint corridor_idx);
};