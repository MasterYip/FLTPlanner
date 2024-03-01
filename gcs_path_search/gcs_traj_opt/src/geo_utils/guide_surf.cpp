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
    : weight_order_(weight_order), weights_(Eigen::VectorXd::Ones(key_points.size())),
      key_points_num_(key_points.size())
{
    key_points_mat_.resize(key_points.size(), 3);
    for (uint i = 0; i < key_points.size(); i++)
    {
        key_points_mat_.row(i) = key_points[i];
    }
}

HarmonicGuideSurf::HarmonicGuideSurf(const Eigen::MatrixX3d &key_points_mat, int weight_order)
    : weight_order_(weight_order), weights_(Eigen::VectorXd::Ones(key_points_mat.rows())),
      key_points_num_(key_points_mat.rows()), key_points_mat_(key_points_mat)
{
}

HarmonicGuideSurf::~HarmonicGuideSurf()
{
}

double HarmonicGuideSurf::getHeight(const Point &p) const
{
    double num = 0;
    double den = 0;
    Eigen::Vector2d p_vec = p.head(2);
    Eigen::VectorXd dists = (key_points_mat_.leftCols(2).rowwise() - p_vec.transpose()).rowwise().norm();
    for (int i = 0; i < key_points_num_; i++)
    {
        if (dists(i) < 1e-6)
            return key_points_mat_(i, 2);
        if (weight_order_ == 1)
        {
            num += weights_(i) / dists(i) * key_points_mat_(i, 2);
            den += weights_(i) / dists(i);
        }
        else
        {
            num += weights_(i) / std::pow(dists(i), weight_order_) * key_points_mat_(i, 2);
            den += weights_(i) / std::pow(dists(i), weight_order_);
        }
    }
    return num / den;
}
