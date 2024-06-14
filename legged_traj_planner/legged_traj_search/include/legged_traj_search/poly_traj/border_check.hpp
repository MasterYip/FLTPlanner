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
#include "legged_traj_search/geo_utils/polycorridor.hpp"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"
#include "legged_traj_search/geo_utils/guide_surf.hpp"
using namespace geo_utils_2d;

class IndexRemap
{
private:
    const grid_map::GridMap &map_;
    grid_map::Size map_size_;
    grid_map::Position map_position_;
    double resolution_;

public:
    IndexRemap(const grid_map::GridMap &map)
        : map_(map),
          map_size_(map.getSize()),
          map_position_(map.getPosition()),
          resolution_(map.getResolution())
    {
    }

    GridPt pos2Grid(const Eigen::Vector2d &pos) const
    {
        GridPt index;
        map_.getIndex(pos, index);
        return index2grid(index);
    }

    Eigen::Vector2d grid2Pos(const GridPt &grid) const
    {
        Eigen::Vector2d pos;
        map_.getPosition(grid2Index(grid), pos);
        return pos;
    }

    GridPt grid2Index(const GridPt &pt) const
    {
        Eigen::Vector2d pos;
        double map_len_x = (map_size_[0] - 1) * resolution_;
        double map_len_y = (map_size_[1] - 1) * resolution_;
        pos[0] = map_position_[0] + 0.5 * map_len_x - pt[0] * resolution_;
        pos[1] = map_position_[1] + 0.5 * map_len_y - pt[1] * resolution_;
        GridPt index;
        map_.getIndex(pos, index);
        return index;
    }

    GridPt index2grid(const GridPt &index) const
    {
        Eigen::Vector2d pos;
        map_.getPosition(index, pos);
        GridPt grid;
        grid[0] = (map_position_[0] + 0.5 * (map_size_[0] - 1) * resolution_ - pos[0]) / resolution_;
        grid[1] = (map_position_[1] + 0.5 * (map_size_[1] - 1) * resolution_ - pos[1]) / resolution_;
        return grid;
    }
};

class BorderCheck
{
private:
    /* data */
    // PROBLEM: Is this safe to use reference here?
    PolyCorridor &poly_corridor_;
    IndexRemap index_remap_;
    const grid_map::GridMap &map_;
    std::string ground_layer_;
    std::string ceiling_layer_;
    bool enable_ceiling_;
    bool enable_ground_;

public:
    BorderCheck(PolyCorridor &poly_corridor,
                const grid_map::GridMap &map,
                const std::string ground_layer = "elevation",
                const std::string ceiling_layer = "ceiling",
                const bool enable_ground = true,
                const bool enable_ceiling = false);

    double queryHeight(const Eigen::Vector2d &pos2d);

    double queryHeight(const GridPt &grid2d);

    /**
     * @brief Check if the given position is in the corridor intersection border
     *
     * @param pos2d Position in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int inBorder(const Eigen::Vector2d &pos2d);

    int inBorder(const GridPt &grid2d);

    bool inCorridor(const GridPt &grid2d, const int &corridor_idx);

    /**
     * @brief Check if the given position is in the polyhedra
     *
     * @param grid2d GridPoint in 2D
     * @param corridor_idx The index of the corridor segment
     * @return int The FIRST index of the polyhedra that the position is in, -1 if not in any segment
     */
    int inPoly(const GridPt &grid2d);

    int inPoly(const GridPt &grid2d, const int &poly_idx);

    const grid_map::GridMap &getMap() const
    {
        return map_;
    }

    const IndexRemap &getIndexRemap() const
    {
        return index_remap_;
    }
};