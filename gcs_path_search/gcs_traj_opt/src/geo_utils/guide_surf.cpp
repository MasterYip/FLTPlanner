/**
 * @file guide_surf.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "gcs_traj_opt/geo_utils/guide_surf.hpp"



HarmonicGuideSurf::HarmonicGuideSurf(const std::vector<Point3D> &key_points, int weight_order)
    : key_points(key_points), weight_order(weight_order)
{
}

HarmonicGuideSurf::~HarmonicGuideSurf()
{
}

double HarmonicGuideSurf::getHeight(const Point &p) const
{
    double height = 0;
    // for (const auto &key_point : key_points)
    // {
    //     double dist = (key_point - p).norm();
    //     height += std::pow(dist, weight_order);
    // }
    return height;
}

