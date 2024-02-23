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
        corridor_.emplace_back(geo_utils::mergeVpoly(polys_.at(i).getVRep(), polys_.at(i + 1).getVRep()));
    }
    // guide plane
    for (uint i = 0; i < poly_size - 1; i++)
    {
        Eigen::Vector3d p1 = polys_.at(i).getInterior();
        Eigen::Vector3d p2 = polys_.at(i + 1).getInterior();
        guide_plane_.emplace_back(geo_utils::getGuidancePlane(p1, p2));
    }
}

PolyCorridor::PolyCorridor(const std::vector<Polyhedra> &polys, const Eigen::Vector3d &start, const Eigen::Vector3d &goal) : PolyCorridor(polys)
{
    if (poly_size == 2)
    {
        guide_plane_.at(0) = geo_utils::getGuidancePlane(start, goal);
    }
    else if (poly_size > 2)
    {
        guide_plane_.at(0) = geo_utils::getGuidancePlane(start, polys_.at(1).getInterior());
        guide_plane_.at(poly_size - 2) = geo_utils::getGuidancePlane(polys_.at(poly_size - 2).getInterior(), goal);
    }
    else
    {
        throw std::invalid_argument("PolyCorridor: poly_size should be at least 2");
    }
}

void PolyCorridor::appendPoly(const Polyhedra &poly)
{
    polys_.emplace_back(poly);
    poly_size++;
    corridor_.emplace_back(geo_utils::mergeVpoly(polys_.at(poly_size - 2).getVRep(), polys_.at(poly_size - 1).getVRep()));
    Eigen::Vector3d p1 = polys_.at(poly_size - 2).getInterior();
    Eigen::Vector3d p2 = polys_.at(poly_size - 1).getInterior();
    guide_plane_.emplace_back(geo_utils::getGuidancePlane(p1, p2));
}

// TODO: faster search algorithm?
int PolyCorridor::isInCorridor(const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < poly_size - 1; i++)
    {
        if (geo_utils::inVpoly(corridor_.at(i).getVRep(), pos))
        {
            return i;
        }
    }
    return -1;
}

int PolyCorridor::isInPoly(const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < poly_size; i++)
    {
        if (geo_utils::inVpoly(polys_.at(i).getVRep(), pos))
        {
            return i;
        }
    }
    return -1;
}

geo_utils::Plain PolyCorridor::getGuidePlain(uint corridor_idx) const
{
    return guide_plane_.at(corridor_idx);
}