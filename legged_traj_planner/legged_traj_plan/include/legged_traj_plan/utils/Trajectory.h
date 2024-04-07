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
#include "legged_traj_search/geo_utils/geo_utils.hpp"
/* internal project header files */

using Point3D = geo_utils::Point3D;

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
    std::vector<Point3D> poly_path_;
    minco::MINCO_S2NU minco_traj_;
    Eigen::Vector3d start_vel_;
    Eigen::Vector3d goal_vel_;
    double total_time_;

    Trajectory<3> traj_;

public:
    MincoTrajectory() = default;

    MincoTrajectory(const std::vector<Point3D> &poly_path,
                    Eigen::Vector3d start_vel = {0, 0, 0.3},
                    Eigen::Vector3d goal_vel = {0, 0, -0.3},
                    double total_time = 1.0) : poly_path_(poly_path),
                                               start_vel_(start_vel),
                                               goal_vel_(goal_vel),
                                               total_time_(total_time)
    {
        Eigen::Matrix<double, 3, 2> start_state;
        start_state.col(0) = poly_path_.front();
        start_state.col(1) = start_vel_;
        Eigen::Matrix<double, 3, 2> goal_state;
        goal_state.col(0) = poly_path_.back();
        goal_state.col(1) = goal_vel_;
        minco_traj_.setConditions(start_state, goal_state, poly_path_.size() - 1);
        if (poly_path_.size() > 2)
        {
            double path_length = 0;
            Eigen::VectorXd ts(poly_path_.size() - 1);
            Eigen::Matrix3Xd inPs(3, poly_path_.size() - 2);
            for (size_t i = 1; i < poly_path_.size() - 1; i++)
            {
                inPs.col(i - 1) = poly_path_[i];
                path_length += (poly_path_[i] - poly_path_[i - 1]).norm();
            }
            path_length += (poly_path_.back() - poly_path_[poly_path_.size() - 2]).norm();
            for (size_t i = 1; i < poly_path_.size(); i++)
            {
                ts[i - 1] = (poly_path_[i] - poly_path_[i - 1]).norm() / path_length * total_time_;
            }
            minco_traj_.setParameters(inPs, ts);
        }
        else
        {
            Eigen::VectorXd ts(1);
            ts << total_time_;
            minco_traj_.setParameters(Eigen::Matrix3Xd::Zero(3, 0), ts);
        }
        updateTraj();
    }

    minco::MINCO_S2NU &getMinco()
    {
        return minco_traj_;
    }

    Trajectory<3> &getTraj()
    {
        return traj_;
    }

    void updateTraj()
    {
        minco_traj_.getTrajectory(traj_);
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

    void getInitCondition(std::vector<Point3D> &poly_path,
                          Eigen::Vector3d &start_vel,
                          Eigen::Vector3d &goal_vel) const
    {
        poly_path = poly_path_;
        start_vel = start_vel_;
        goal_vel = goal_vel_;
    }

    // Test
    bool getTrajSamples(std::vector<Point3D> &discrete_traj, double T = 0.01, bool update = false)
    {
        if (update)
            updateTraj();
        if (traj_.getPieceNum() == 0)
        {
            return false;
        }
        discrete_traj.clear();
        Eigen::Vector3d lastX = traj_.getPos(0.0);
        for (double t = T; t < traj_.getTotalDuration(); t += T)
        {
            discrete_traj.emplace_back(traj_.getPos(t));
        }
        return true;
    }
};
