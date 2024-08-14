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
     * @param t interpolation parameter
     * @param d_order derivative order (0 for position, 1 for velocity, etc.)
     * @param normalized whether to use normalized parameter t
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false)
    {
        throw std::runtime_error("Not implemented");
        return Eigen::VectorXd::Zero(3);
    }

    virtual double getTotalDuration() const
    {
        printf("getTotalDuration() not implemented\n");
        return 0;
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
    bool space_deform_flag_{false};
    Eigen::Vector3d space_deform_{Eigen::Vector3d::Ones()};

public:
    MincoTrajectory() = default;

    MincoTrajectory(const std::vector<Point3D> &poly_path,
                    const Eigen::VectorXd &ts,
                    Eigen::Vector3d start_vel = {0, 0, 0},
                    Eigen::Vector3d goal_vel = {0, 0, 0},
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
            Eigen::Matrix3Xd inPs(3, poly_path_.size() - 2);
            for (size_t i = 1; i < poly_path_.size() - 1; i++)
            {
                inPs.col(i - 1) = poly_path_[i];
            }
            minco_traj_.setParameters(inPs, ts);
        }
        else
        {
            minco_traj_.setParameters(Eigen::Matrix3Xd::Zero(3, 0), ts);
        }
        updateTraj();
    }

    MincoTrajectory(const std::vector<Point3D> &poly_path,
                    Eigen::Vector3d start_vel = {0, 0, 0},
                    Eigen::Vector3d goal_vel = {0, 0, 0},
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

    bool setConditions(Eigen::Vector3d start, Eigen::Vector3d end,
                       Eigen::Vector3d start_vel, Eigen::Vector3d end_vel)
    {
        Eigen::Matrix<double, 3, 2> start_state;
        start_state.col(0) = start;
        start_state.col(1) = start_vel;
        Eigen::Matrix<double, 3, 2> goal_state;
        goal_state.col(0) = end;
        goal_state.col(1) = end_vel;
        minco_traj_.setConditions(start_state, goal_state, poly_path_.size() - 1); // PROBLEM: Will Reseting causing failure?
        updateTraj();
        return true;
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

    void updateTraj(const Trajectory<3> &traj)
    {
        traj_ = traj;
    }

    Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) override
    {
        if (traj_.getPieceNum() == 0)
            throw std::runtime_error("Trajectory is empty");
        double multiply_factor = 1.0;
        if (normalized)
        {
            t *= traj_.getTotalDuration();
            // BUG: derivative depends on total_time_
            multiply_factor = traj_.getTotalDuration();
        }

        // Limit
        if (t < 0)
            t = 0;
        else if (t > traj_.getTotalDuration())
            t = traj_.getTotalDuration();

        if (d_order == 0)
            return space_deform_flag_ ? traj_.getPos(t).cwiseProduct(space_deform_) : traj_.getPos(t);
        else if (d_order == 1)
            return space_deform_flag_ ? traj_.getVel(t).cwiseProduct(space_deform_) : traj_.getVel(t);
        else if (d_order == 2)
            return space_deform_flag_ ? traj_.getAcc(t).cwiseProduct(space_deform_) : traj_.getAcc(t);
        else if (d_order == 3)
            return space_deform_flag_ ? traj_.getJer(t).cwiseProduct(space_deform_) : traj_.getJer(t);
        else
            throw std::runtime_error("Invalid derivative order");
    }

    double getTotalDuration() const override
    {
        return traj_.getTotalDuration();
    }

    void getInitCondition(std::vector<Point3D> &poly_path,
                          Eigen::Vector3d &start_vel,
                          Eigen::Vector3d &goal_vel) const
    {
        poly_path = poly_path_;
        start_vel = start_vel_;
        goal_vel = goal_vel_;
    }

    void getOptInitCondition(std::vector<Point3D> &poly_path,
                             Eigen::Vector3d &start_vel,
                             Eigen::Vector3d &goal_vel,
                             const double &max_piece_length) const
    {
        double tot_length = 0;
        for (int i = 1; i < poly_path_.size(); i++)
            tot_length += (poly_path_[i] - poly_path_[i - 1]).norm();
        int seg_num = tot_length / max_piece_length > 1 ? tot_length / max_piece_length : 1;
        for (int i = 0; i < seg_num + 2; i++)
            poly_path.emplace_back(traj_.getPos((double)i / (seg_num + 1) * traj_.getTotalDuration()));
        start_vel = start_vel_;
        goal_vel = goal_vel_;
    }

    // Test
    bool getTrajSamples(std::vector<Point3D> &discrete_traj, double T = 0.01, bool normalized = true)
    {
        discrete_traj.clear();
        if (traj_.getPieceNum() == 0)
            return false;
        double delta = normalized ? T * traj_.getTotalDuration() : T;
        for (double t = 0; t < traj_.getTotalDuration(); t += delta)
            discrete_traj.emplace_back(space_deform_flag_ ? traj_.getPos(t).cwiseProduct(space_deform_) : traj_.getPos(t));
        return true;
    }

    void setSpaceDeform(const Eigen::Vector3d &space_deform)
    {
        space_deform_flag_ = true;
        space_deform_ = space_deform;
    }

    void unsetSpaceDeform()
    {
        space_deform_flag_ = false;
        space_deform_ = Eigen::Vector3d::Ones();
    }
};
