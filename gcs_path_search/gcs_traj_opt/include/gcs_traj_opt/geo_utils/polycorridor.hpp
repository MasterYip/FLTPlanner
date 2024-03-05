/**
 * @file polycorridor.hpp
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
#include "geo_utils.hpp"
#include "polyhedra.hpp"
#include "guide_surf.hpp"

// struct PointLocation
// {
//     int poly_idx;
//     int corridor_idx;
// }

using namespace geo_utils;

class PolyCorridor
{
private:
    uint poly_size_ = 0;
    Point3D start_;
    Point3D goal_;

    std::vector<Polyhedra> polys_;    // discret feasible polyhedra trajectory (size n)
    std::vector<Polyhedra> corridor_; // feasible corridor obtained by `poly merging` (size n-1)
    HarmonicGuideSurf guide_surf_;

public:
    /**
     * @brief Construct a new Poly Corridor and generate guide_plane using INTERIOR POINTs of the polyhedra
     *
     * @param polys
     */
    PolyCorridor(const std::vector<Polyhedra> &polys);
    /**
     * @brief Construct a new Poly Corridor and generate guide_plane using the START and GOAL(for the first & last guide plain) and INTERIOR POINTs
     *
     * @param polys
     * @param start
     * @param goal
     */
    PolyCorridor(const std::vector<Polyhedra> &polys, const Point3D &start, const Point3D &goal);

    /**
     * @brief Check if the given position is in the corridor
     *
     * @param pos
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int isInCorridor(const Eigen::Vector3d &pos);

    bool isInCorridor(const Eigen::Vector3d &pos, const int &corridor_idx);

    int isInPoly(const Eigen::Vector3d &pos);

    bool isInPoly(const Eigen::Vector3d &pos, const int &poly_idx);

    uint getPolySize() const
    {
        return poly_size_;
    };

    uint getCorridorSize() const
    {
        return poly_size_ - 1;
    };

    std::vector<Polyhedra> &getPolys()
    {
        return polys_;
    };

    std::vector<Polyhedra> &getCorridor()
    {
        return corridor_;
    };

    HarmonicGuideSurf &getGuideSurf()
    {
        return guide_surf_;
    };

    Point3D getStart() const
    {
        return start_;
    };

    Point3D getGoal() const
    {
        return goal_;
    };
};
