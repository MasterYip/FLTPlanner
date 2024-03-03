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
                         const grid_map::GridMap &map,
                         const std::string ground_layer,
                         const bool enable_ceiling,
                         const std::string ceiling_layer)
    : poly_corridor_(poly_corridor),
      map_(map),
      ground_layer_(ground_layer),
      enable_ceiling_(enable_ceiling),
      ceiling_layer_(ceiling_layer)
{
}


double BorderCheck::queryHeight(const Eigen::Vector2d &pos2d)
{
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (query_height < map_.atPosition(ground_layer_, pos2d))
    {
        query_height = map_.atPosition(ground_layer_, pos2d);
    }
    if (enable_ceiling_ && query_height > map_.atPosition(ceiling_layer_, pos2d))
    {
        query_height = map_.atPosition(ceiling_layer_, pos2d);
    }
    return query_height;
}

double BorderCheck::queryHeight(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    map_.getPosition(grid2d, pos2d);
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (query_height < map_.at(ground_layer_, grid2d))
    {
        query_height = map_.at(ground_layer_, grid2d);
    }
    if (enable_ceiling_ && query_height > map_.at(ceiling_layer_, grid2d))
    {
        query_height = map_.at(ceiling_layer_, grid2d);
    }
    return query_height;
}

int BorderCheck::inBorder(const Eigen::Vector2d &pos2d)
{
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(pos2d)));
}

int BorderCheck::inBorder(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    map_.getPosition(grid2d, pos2d);
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)));
}

int BorderCheck::inPoly(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    map_.getPosition(grid2d, pos2d);
    return poly_corridor_.isInPoly(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)));
}