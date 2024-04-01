/**
 * @file Trajectory.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <Eigen/Dense>
#include "legged_traj_search/geo_utils/minco.hpp"
/* internal project header files */

class TrajectoryBase
{
public:
    TrajectoryBase() = default;

    /**
     * @brief Evaluate the trajectory at t.
     *
     * @param t normalized interpolation parameter
     * @param d_order derivative order (0 for position, 1 for velocity, etc.)
     * @param normalized whether to use normalized parameter t
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false)
    {
        throw std::runtime_error("Not implemented");
        return Eigen::VectorXd::Zero(3);
    }
};

class MincoTrajectory : public TrajectoryBase
{
private:
    Trajectory<3> traj_;

public:
    MincoTrajectory() = default;
    MincoTrajectory(const Trajectory<3> traj)
    {
        traj_ = std::move(traj);
    }
    MincoTrajectory(const minco::MINCO_S2NU &minco)
    {
        minco.getTrajectory(traj_);
    }

    Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) override
    {
        if (traj_.getPieceNum() == 0)
            throw std::runtime_error("Trajectory is empty");

        if (normalized)
            t *= traj_.getTotalDuration();

        if (t < 0 || t > traj_.getTotalDuration())
            throw std::runtime_error("Invalid time");

        if (d_order == 0)
            return traj_.getPos(t);
        else if (d_order == 1)
            return traj_.getVel(t);
        else if (d_order == 2)
            return traj_.getAcc(t);
        else if (d_order == 3)
            return traj_.getJer(t);
        else
            throw std::runtime_error("Invalid derivative order");
    }
};
