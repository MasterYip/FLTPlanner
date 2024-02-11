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

class PolyCorridor
{
private:
    uint poly_size = 0;
    std::vector<Polyhedra> polys_;             // discret feasible polyhedra trajectory
    std::vector<Polyhedra> corridor_;          // feasible corridor obtained by `convex hull merging`
    std::vector<Eigen::Vector4d> guide_plane_; // guide plane for each corridor segment
public:
    PolyCorridor(const std::vector<Polyhedra> &polys);

    void appendPoly(const Polyhedra &poly);
};
