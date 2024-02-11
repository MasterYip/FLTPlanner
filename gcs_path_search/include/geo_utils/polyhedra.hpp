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
    Polyhedra(const Eigen::Matrix3Xd vpoly)
    {
        v_rep_ = vpoly;
        v_rep_valid = true;
    }
    Polyhedra(const Eigen::MatrixX4d hpoly)
    {
        if (findInterior(hPoly, inner_))
        {
            h_rep_ = hpoly;
            h_rep_valid = true;
        }
        else
        {
            throw std::runtime_error("No interior point found");
        }
    }

    Eigen::Matrix3Xd getVRep()
    {
        if (v_rep_valid)
            return v_rep_;
        else
        {
            if (h_rep_valid)
            {
                geo_utils::enumerateVs(h_rep_, inner_, v_rep_, enum_eps_);
                v_rep_valid = true;
                return v_rep_;
            }
            else
            {
                throw std::runtime_error("No valid representation");
            }
        }
    }

    Eigen::MatrixX4d getHRep()
    {
        if (h_rep_valid)
            return h_rep_;
        else
        {
            if (v_rep_valid)
            {
                h_rep_ = geo_utils::vpoly2hpoly(v_rep_);
                h_rep_valid = true;
                return h_rep_;
            }
            else
            {
                throw std::runtime_error("No valid representation");
            }
        }
    }

    Eigen::Vector3d getInterior()
    {
        if (h_rep_valid)
            return inner_;
        else
        {
            if (v_rep_valid)
            {
                h_rep_ = geo_utils::vpoly2hpoly(v_rep_);
                h_rep_valid = true;
                geo_utils::findInterior(h_rep_, inner_); // TODO: check if this is false
                return inner_;
            }
            else
            {
                throw std::runtime_error("No valid representation");
            }
        }
    }
};