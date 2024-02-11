/**
 * @file polyhedra.hpp
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
#include <Eigen/Core>
/* internal project header files */
#include "geo_utils/geo_utils.hpp"

class Polyhedra
{
private:
    Eigen::MatrixX4d h_rep_;
    Eigen::Vector3d inner_;   // Interior point
    bool h_rep_valid = false; // both h_rep_ & inner_ are valid if true
    Eigen::Matrix3Xd v_rep_;
    bool v_rep_valid = false;

    double enum_eps_ = 1.0e-6;

public:
    /**
     * @brief Construct from vertex representation
     * @note vpoly may have redundant vertices (inner points)
     * @param vpoly
     */
    Polyhedra(const Eigen::Matrix3Xd vpoly);
    Polyhedra(const Eigen::MatrixX4d hpoly);

    Eigen::Matrix3Xd getVRep();
    Eigen::MatrixX4d getHRep();
    Eigen::Vector3d getInterior();
};
