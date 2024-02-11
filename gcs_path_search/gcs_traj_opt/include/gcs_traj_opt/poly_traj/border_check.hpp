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

class BorderCheck
{
private:
    /* data */
    grid_map::GridMap map_ground_;
    grid_map::GridMap map_ceiling_;
    PolyCorridor poly_corridor_;
public:
    BorderCheck(const grid_map::GridMap &map_ground, const grid_map::GridMap &map_ceiling, const PolyCorridor &poly_corridor);
}