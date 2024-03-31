/**
 * @file polyhedra.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_search/geo_utils/polyhedra.hpp"

Polyhedra::Polyhedra(const Eigen::Matrix3Xd vpoly)
{
    v_rep_ = vpoly;
    v_rep_valid = true;
}

Polyhedra::Polyhedra(const Eigen::MatrixX4d hpoly)
{
    if (geo_utils::findInterior(hpoly, inner_))
    {
        h_rep_ = hpoly;
        h_rep_valid = true;
    }
    else
    {
        throw std::runtime_error("No interior point found");
    }
}

Eigen::Matrix3Xd Polyhedra::getVRep()
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

Eigen::MatrixX4d Polyhedra::getHRep()
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

Eigen::Vector3d Polyhedra::getInterior()
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
