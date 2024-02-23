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

BorderCheck::BorderCheck(PolyCorridor &poly_corridor,
                         const grid_map::GridMap &map_ground,
                         const grid_map::GridMap &map_ceiling,
                         const std::string map_layer)
    : map_ground_(map_ground), map_ceiling_(map_ceiling), poly_corridor_(poly_corridor), map_layer_(map_layer)
{
}

int BorderCheck::inBorder(const Eigen::Vector2d &pos2d, uint corridor_idx)
{
    geo_utils::Plain guide_plain = poly_corridor_.getGuidePlain(corridor_idx);
    // FIXME: c = 0?
    double query_height = (-guide_plain(3) - guide_plain(0) * pos2d(0) - guide_plain(1) * pos2d(1)) / guide_plain(2);
    // Saturation
    if (query_height < map_ground_.atPosition(map_layer_, pos2d))
    {
        query_height = map_ground_.atPosition(map_layer_, pos2d);
    }
    if (query_height > map_ceiling_.atPosition(map_layer_, pos2d))
    {
        query_height = map_ceiling_.atPosition(map_layer_, pos2d);
    }
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), query_height));
}

int BorderCheck::inBorder(const GridPt &grid2d, uint corridor_idx)
{
    geo_utils::Plain guide_plain = poly_corridor_.getGuidePlain(corridor_idx);
    Eigen::Vector2d pos2d;
    map_ground_.getPosition(grid2d, pos2d);
    double query_height = (-guide_plain(3) - guide_plain(0) * pos2d(0) - guide_plain(1) * pos2d(1)) / guide_plain(2);
    if (query_height < map_ground_.at(map_layer_, grid2d))
    {
        query_height = map_ground_.at(map_layer_, grid2d);
    }
    if (query_height > map_ceiling_.at(map_layer_, grid2d))
    {
        query_height = map_ceiling_.at(map_layer_, grid2d);
    }
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), query_height));
}

int BorderCheck::inPoly(const GridPt &grid2d, uint corridor_idx)
{
    geo_utils::Plain guide_plain = poly_corridor_.getGuidePlain(corridor_idx);
    Eigen::Vector2d pos2d;
    map_ground_.getPosition(grid2d, pos2d);
    double query_height = (-guide_plain(3) - guide_plain(0) * pos2d(0) - guide_plain(1) * pos2d(1)) / guide_plain(2);
    if (query_height < map_ground_.at(map_layer_, grid2d))
    {
        query_height = map_ground_.at(map_layer_, grid2d);
    }
    if (query_height > map_ceiling_.at(map_layer_, grid2d))
    {
        query_height = map_ceiling_.at(map_layer_, grid2d);
    }
    return poly_corridor_.isInPoly(Eigen::Vector3d(pos2d(0), pos2d(1), query_height));
}