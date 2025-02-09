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
#include <grid_map_core/grid_map_core.hpp>
/* internal project header files */
#include "legged_traj_search/geo_utils/geo_utils.hpp"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils;
using namespace geo_utils_2d;

class BaseGuideSurf
{
public:
    BaseGuideSurf() = default;
    virtual ~BaseGuideSurf() = default;
    virtual double getHeight(const Point &p) const { return 0; };
};

class HarmonicGuideSurf : public BaseGuideSurf
{
private:
    int key_points_num_;
    Eigen::MatrixX3d key_points_mat_; // (n points, 3)
    int weight_order_;
    Eigen::VectorXd weights_;

public:
    HarmonicGuideSurf() = default;
    /**
     * @brief Construct a new Harmonic Guide Surf object
     *
     * @param key_points 3D points
     * @param weight_order default 1
     */
    HarmonicGuideSurf(const std::vector<Point3D> &key_points, int weight_order = 1);
    HarmonicGuideSurf(const Eigen::MatrixX3d &key_points_mat, int weight_order = 1);
    ~HarmonicGuideSurf();

    double getHeight(const Point &p) const;
};

class ConvolutedGuideSurf : public BaseGuideSurf
{
private:
    const grid_map::GridMap &map_;
    std::string ground_layer_;
    int kernel_size_;
    double kernel_interval_;

public:
    /**
     * @brief Construct a new Convoluted Guide Surf object
     *
     * @param map
     * @param kernel_size       odd number
     * @param kernel_interval   in meter
     * @param ground_layer      layer name of the ground
     */
    ConvolutedGuideSurf(const grid_map::GridMap &map,
                        int kernel_size = 3,
                        double kernel_interval = 0.1,
                        const std::string ground_layer = "elevation")
        : map_(map),
          kernel_size_(kernel_size),
          kernel_interval_(kernel_interval),
          ground_layer_(ground_layer)
    {
    }
    double getHeight(const Point &p) const
    {
        double height_sum = 0;
        for (int i = -kernel_size_ / 2; i <= kernel_size_ / 2; i++)
        {
            for (int j = -kernel_size_ / 2; j <= kernel_size_ / 2; j++)
            {
                Eigen::Vector2d pos2d;
                pos2d << p(0) + i * kernel_interval_, p(1) + j * kernel_interval_;
                try
                {
                    height_sum += map_.atPosition(ground_layer_, pos2d);
                }
                catch (const std::out_of_range &e)
                {
                    continue;
                }
            }
        }
        return height_sum / (kernel_size_ * kernel_size_);
    }
};
