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

inline int general_mod(int a, int b)
{
    return (a % b + b) % b;
}

/**
 * @brief IndexRemap class
 * @note Index: index of the grid map, usually fixed to `world`
 *       Grid: index of the grid map, usually fixed to `map`
 * more details of the gridmap see
 * https://github.com/ANYbotics/grid_map/raw/master/grid_map_core/doc/grid_map_conventions.png
 */
class IndexRemap
{
private:
    const grid_map::GridMap &map_;
    grid_map::Size map_size_;
    grid_map::Position map_position_;
    double resolution_;
    Eigen::Array2i map_shift_ = {0, 0};

public:
    IndexRemap(const grid_map::GridMap &map)
        : map_(map),
          map_size_(map.getSize()),
          map_position_(map.getPosition()),
          resolution_(map.getResolution())
    {
        // IMPORTANT: If map.setPosition() is called, map_shift should be zero.
        // map_shift_ is designed for map.move().
        GridPt index;
        map_.getIndex(map_position_, index);
        if (!(index[0] * 2 == map_size_[0] && index[1] * 2 == map_size_[1]))
        {
            map_shift_[0] = (int)(map_position_[0] / resolution_);
            map_shift_[1] = (int)(map_position_[1] / resolution_);
        }
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
        return {general_mod(pt[0] - map_shift_[0], map_size_[0]),
                general_mod(pt[1] - map_shift_[1], map_size_[1])};
    }

    GridPt index2grid(const GridPt &index) const
    {
        return {general_mod(index[0] + map_shift_[0], map_size_[0]),
                general_mod(index[1] + map_shift_[1], map_size_[1])};
    }
};

class BorderCheckBase
{
public:
    virtual double queryHeight(const Eigen::Vector2d &pos2d) = 0;

    virtual double queryHeight(const GridPt &grid2d) = 0;

    virtual double disInBorder(const Eigen::Vector2d &pos2d) = 0;

    virtual double disInBorder(const GridPt &grid2d) = 0;

    virtual bool isStartValid(const GridPt &start) = 0;

    virtual bool isGoalValid(const GridPt &goal) = 0;
};

class CorridorBorderCheck : public BorderCheckBase
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

public:
    CorridorBorderCheck(PolyCorridor &poly_corridor,
                        const grid_map::GridMap &map,
                        const std::string ground_layer = "elevation",
                        const std::string ceiling_layer = "ceiling",
                        const bool enable_ground = true,
                        const bool enable_ceiling = false);

    double queryHeight(const Eigen::Vector2d &pos2d) override;

    double queryHeight(const GridPt &grid2d) override;

    bool isStartValid(const GridPt &start) override
    {
        return inPoly(start, 0);
    }

    bool isGoalValid(const GridPt &goal) override
    {
        return inPoly(goal, poly_corridor_.getPolySize() - 1);
    }

    double disInBorder(const Eigen::Vector2d &pos2d) override;

    double disInBorder(const GridPt &grid2d) override;
};
