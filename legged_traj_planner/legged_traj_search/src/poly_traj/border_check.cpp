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

#include "legged_traj_search/poly_traj/border_check.hpp"

BorderCheck::BorderCheck(PolyCorridor &poly_corridor,
                         const grid_map::GridMap &map,
                         const std::string ground_layer,
                         const std::string ceiling_layer,
                         const bool enable_ground,
                         const bool enable_ceiling)
    : poly_corridor_(poly_corridor),
      map_(map),
      index_remap_(map),
      ground_layer_(ground_layer),
      ceiling_layer_(ceiling_layer),
      enable_ceiling_(enable_ceiling),
      enable_ground_(enable_ground)
{
}

double BorderCheck::queryHeight(const Eigen::Vector2d &pos2d)
{
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (enable_ground_ && query_height < map_.atPosition(ground_layer_, pos2d))
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
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (enable_ground_ && query_height < map_.at(ground_layer_, index))
    {
        query_height = map_.at(ground_layer_, index);
    }
    if (enable_ceiling_ && query_height > map_.at(ceiling_layer_, index))
    {
        query_height = map_.at(ceiling_layer_, index);
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
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)));
}

double BorderCheck::disInBorder(const Eigen::Vector2d &pos2d)
{
    return -poly_corridor_.disOutCorrider(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(pos2d)));
}

double BorderCheck::disInBorder(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    return -poly_corridor_.disOutCorrider(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)));
}

bool BorderCheck::inCorridor(const GridPt &grid2d, const int &corridor_idx)
{
    Eigen::Vector2d pos2d;
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    return poly_corridor_.isInCorridor(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)), corridor_idx);
}

int BorderCheck::inPoly(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    return poly_corridor_.isInPoly(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)));
}

int BorderCheck::inPoly(const GridPt &grid2d, const int &poly_idx)
{
    Eigen::Vector2d pos2d;
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    return poly_corridor_.isInPoly(Eigen::Vector3d(pos2d(0), pos2d(1), queryHeight(grid2d)), poly_idx);
}