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

// struct PointLocation
// {
//     int poly_idx;
//     int corridor_idx;
// }

class PolyCorridor
{
private:
    uint poly_size = 0;
    std::vector<Polyhedra> polys_;             // discret feasible polyhedra trajectory (size n)
    std::vector<Polyhedra> corridor_;          // feasible corridor obtained by `poly merging` (size n-1)
    std::vector<Eigen::Vector4d> guide_plane_; // guide plane for each corridor segment (size n-1)
public:
    PolyCorridor(const std::vector<Polyhedra> &polys);

    void appendPoly(const Polyhedra &poly);

    /**
     * @brief Check if the given position is in the corridor
     *
     * @param pos
     * @return int The FIRST index of the corridor segment that the position is in, -1 if not in any segment
     */
    int isInCorridor(const Eigen::Vector3d &pos);

    int isInPoly(const Eigen::Vector3d &pos);

    geo_utils::Plain getGuidePlain(uint corridor_idx) const;

    uint getPolySize() const
    {
        return poly_size;
    };

    uint getCorridorSize() const
    {
        return poly_size - 1;
    };

    std::vector<Polyhedra> getPolys() const
    {
        return polys_;
    };

    std::vector<Polyhedra> getCorridor() const
    {
        return corridor_;
    };
};
