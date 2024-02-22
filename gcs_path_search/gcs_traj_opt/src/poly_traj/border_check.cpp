/**
 * @file border_check.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "gcs_traj_opt/poly_traj/border_check.hpp"

BorderCheck::BorderCheck(const grid_map::GridMap &map_ground, const grid_map::GridMap &map_ceiling, const PolyCorridor &poly_corridor)
    : map_ground_(map_ground), map_ceiling_(map_ceiling), poly_corridor_(poly_corridor)
{
}

bool BorderCheck::inBorder(const Eigen::Vector3d &pos)
{
    // check if the position is in the border
    // ...
    return true;
}