/**
 * @file minco_trajopt.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-29
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
#include "gcs_traj_opt/geo_utils/minco.hpp"
#include "gcs_traj_opt/geo_utils/trajectory.hpp"

class MincoTrajOpt
{
private:
    std::vector<Point3D> poly_path_;
    minco::MINCO_S2NU minco_traj_;
    Eigen::Vector3d start_vel_ = {0, 0, 0.3};
    Eigen::Vector3d goal_vel_ = {0, 0, -0.3};
    double total_time_ = 1.0;

public:
    MincoTrajOpt(const std::vector<Point3D> &poly_path)
        : poly_path_(poly_path)
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
    }

    bool getTrajSamples(std::vector<Point3D> &discrete_traj, double T = 0.01)
    {
        Trajectory<3> traj;
        minco_traj_.getTrajectory(traj);
        if (traj.getPieceNum() == 0)
        {
            return false;
        }
        discrete_traj.clear();
        Eigen::Vector3d lastX = traj.getPos(0.0);
        for (double t = T; t < traj.getTotalDuration(); t += T)
        {
            discrete_traj.emplace_back(traj.getPos(t));
        }
        return true;
    }
};