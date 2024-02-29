/**
 * @file guide_surf.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */

/* internal project header files */
#include "gcs_traj_opt/geo_utils/geo_utils.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils;
using namespace geo_utils_2d;

class HarmonicGuideSurf
{
private:
    int key_points_num_;
    std::vector<Point3D> key_points_;
    Eigen::MatrixX3d key_points_mat_;
    int weight_order_;
    Eigen::VectorXd weights_;

public:
    HarmonicGuideSurf(const std::vector<Point3D> &key_points, int weight_order = 1);
    ~HarmonicGuideSurf();

    double getHeight(const Point &p) const;
};
