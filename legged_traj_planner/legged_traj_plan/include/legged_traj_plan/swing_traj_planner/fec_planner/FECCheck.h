/**
 * @file FECCheck.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-01-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/utils/Trajectory.h"
#include "legged_traj_plan/utils/Geometry.h"

struct FECCheckConfig
{
    // double collBallRad1 = 0.0;
    double collBallRad2 = 0.0; // Knee
    double collBallRad3 = 0.0; // Foot
    double FootCollExcludeBallRad = 0.0;

    int checkResolution = 100;

    FECCheckConfig() = default;
};

class FECCheck
{
private:
    std::shared_ptr<BaseRobotInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    FECCheckConfig config_;

public:
    FECCheck(std::shared_ptr<BaseRobotInterface> robot_interface,
             std::shared_ptr<GridMapInterface> gridmap_interface,
             const FECCheckConfig &config = FECCheckConfig()) : robot_interface_(robot_interface),
                                                                gridmap_interface_(gridmap_interface),
                                                                config_(config)
    {
    }

    bool setConfig(const FECCheckConfig &config)
    {
        config_ = config;
        return true;
    }

    bool checkLegFEC(const pinocchio::SE3 &pose, const Eigen::Vector3d &p,
                     const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index)
    {
        // Check if the leg state is valid according to FEC
        // 1. Check if the foot is in the task space
        Eigen::Vector3d pos = point_SE3Act(pose, p);
        Eigen::Vector3d sol;
        bool joint_limit_check = robot_interface_->IKFast_foot(pos, sol, index);
        if (!joint_limit_check)
            return false;
        // 2. Check if the leg is in the collision-free space
        Eigen::Vector3d knee_pos = point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(sol, index, 2));
        bool collision_check_knee = config_.collBallRad2 < gridmap_interface_->sdfValue(knee_pos, "min");
        if (!collision_check_knee)
            return false;
        // 3. Check if the foot is in the collision-free space
        if (!inSphere(p, p0, config_.FootCollExcludeBallRad) && !inSphere(p, p1, config_.FootCollExcludeBallRad))
        {
            bool collision_check_foot = config_.collBallRad3 < gridmap_interface_->sdfValue(p, "min");
            if (!collision_check_foot)
                return false;
        }
        return true;
    }

    bool checkTrajReachability(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                               const std::shared_ptr<TrajectoryBase> &traj,
                               const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                               const bool cfg_space = false)
    {
        double detla = 1.0 / config_.checkResolution;
        double t = 0.0;
        if (!checkLegFEC(pose0, p0, p0, p1, index) ||
            !checkLegFEC(pose1, p1, p0, p1, index))
            return false;
        while (t <= 1.0)
        {
            Eigen::Vector3d p = traj->evaluate(t, 0, true);
            pinocchio::SE3 pose = poseLinearInterp(pose0, pose1, t);
            if (cfg_space)
                p = point_SE3Act(pose.inverse(), robot_interface_->FK_foot(p, index));
            if (!checkLegFEC(pose, p, p0, p1, index))
                return false;
            t += detla;
        }
        return true;
    }
};