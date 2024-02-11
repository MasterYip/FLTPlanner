/**
 * @file polycorridor.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "gcs_traj_opt/geo_utils/polycorridor.hpp"

PolyCorridor::PolyCorridor(const std::vector<Polyhedra> &polys) : polys_(polys)
{
    poly_size = polys.size();
    // poly corridor
    for (uint i = 0; i < poly_size - 1; i++)
    {
        corridor_.emplace_back(geo_utils::mergeVpoly(polys_.at(i), polys_.at(i + 1)));
    }
    // guide plane
    for (uint i = 0; i < poly_size - 1; i++)
    {
        Eigen::Vector3d p1 = polys_.at(i).getInterior();
        Eigen::Vector3d p2 = polys_.at(i + 1).getInterior();
        guide_plane_.emplace_back(geo_utils::getGuidancePlane(p1, p2));
    }
}

// FIXME: not needed
void PolyCorridor::appendPoly(const Polyhedra &poly)
{
    polys_.emplace_back(poly);
    poly_size++;
}

int PolyCorridor::isInCorridor(const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < corridor_.size(); i++)
    {
        if (geo_utils::inVpoly(corridor_.at(i), pos))
        {
            return i;
        }
    }
    return -1;
}