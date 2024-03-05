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
    poly_size_ = polys.size();
    // poly corridor
    for (uint i = 0; i < poly_size_ - 1; i++)
    {
        corridor_.emplace_back(geo_utils::mergeVpoly(polys_.at(i).getVRep(), polys_.at(i + 1).getVRep()));
    }
    // guide surf
    std::vector<geo_utils::Point3D> key_points;
    for (uint i = 0; i < poly_size_; i++)
    {
        key_points.emplace_back(polys_.at(i).getInterior());
    }
    guide_surf_ = HarmonicGuideSurf(key_points);
}

PolyCorridor::PolyCorridor(const std::vector<Polyhedra> &polys,
                           const Point3D &start, const Point3D &goal) : PolyCorridor(polys)
{
    start_ = start;
    goal_ = goal;
    // guide surf
    if (poly_size_ >= 2)
    {

        std::vector<geo_utils::Point3D> key_points;
        key_points.emplace_back(start);
        for (uint i = 1; i < poly_size_ - 1; i++)
        {
            key_points.emplace_back(polys_.at(i).getInterior());
        }
        key_points.emplace_back(goal);
        guide_surf_ = HarmonicGuideSurf(key_points);
    }
    else
    {
        throw std::invalid_argument("PolyCorridor: poly_size_ should be at least 2");
    }
}

// TODO: faster search algorithm?
int PolyCorridor::isInCorridor(const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < poly_size_ - 1; i++)
    {
        if (geo_utils::inHpoly(corridor_.at(i).getHRep(), pos))
        {
            return i;
        }
    }
    return -1;
}

bool PolyCorridor::isInCorridor(const Eigen::Vector3d &pos, const int &corridor_idx)
{
    if (corridor_idx < 0 || corridor_idx >= poly_size_ - 1)
    {
        // std::cerr << "PolyCorridor: corridor_idx out of range" << std::endl;
        return false;
    }
    return geo_utils::inHpoly(corridor_.at(corridor_idx).getHRep(), pos);
}

int PolyCorridor::isInPoly(const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < poly_size_; i++)
    {
        if (geo_utils::inHpoly(polys_.at(i).getHRep(), pos))
        {
            return i;
        }
    }
    return -1;
}

bool PolyCorridor::isInPoly(const Eigen::Vector3d &pos, const int &poly_idx)
{
    if (poly_idx < 0 || poly_idx >= poly_size_)
    {
        return false;
    }
    return geo_utils::inHpoly(polys_.at(poly_idx).getHRep(), pos);
}
