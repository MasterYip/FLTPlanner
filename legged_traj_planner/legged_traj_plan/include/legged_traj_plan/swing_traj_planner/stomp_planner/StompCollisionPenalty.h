/**
 * @file StompCollisionPenalty.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "legged_traj_search/utils/gcs_visualizer.hpp"
/* internal project header files */
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/Utils.h"

/**
 * @brief Collision Penalty for cartesian space (foot)
 *
 */
class StompCollisionPenalty
{
private:
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    double collBallRadius_;
    double weight_;
    double mu_;

    double endCollExcludeRadius_; // TODO: use gaussian weight better
    Eigen::Vector3d startExcludeBall_;
    Eigen::Vector3d endExcludeBall_;

    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    StompCollisionPenalty(std::shared_ptr<GridMapInterface> gridmap_interface,
                          std::shared_ptr<GCSVisualizer> visualizer = nullptr)
        : gridmap_interface_(gridmap_interface)
    {
        if (visualizer != nullptr)
        {
            enable_vis_ = true;
            visualizer_ = visualizer;
        }
    }

    void setupVis(std::shared_ptr<GCSVisualizer> visualizer)
    {
        if (visualizer != nullptr)
        {
            enable_vis_ = true;
            visualizer_ = visualizer;
        }
    }

    void setupParams(SwingTrajPlannerConfig &config)
    {
        collBallRadius_ = config.CollBall3Rad;
        weight_ = config.CollBall3Weight;
        mu_ = config.smoothingFactor;
        endCollExcludeRadius_ = config.FootCollExcludeBallRad;
    }

    /**
     * @brief Setup excluding ball for collision checking (cartesian space)
     *
     * @param start
     * @param end
     */
    void setExcludeBall(const Eigen::Vector3d start, const Eigen::Vector3d end)
    {
        startExcludeBall_ = start;
        endExcludeBall_ = end;
    }

    /**
     * @brief Attach penalty to position, velocity and acceleration
     * FIXME: This gradient is not correct
     * @param pos Position in world frame
     * @param pena Penalty
     * @return if collision happens
     */
    bool attachPena(const Eigen::Vector3d &pos,
                    double &pena)
    {
        // WORLD frame
        Eigen::Vector3d sdfGrad;
        double sdf = gridmap_interface_->sdfValue(pos, "min");
        double f, df;
        if ((pos - startExcludeBall_).norm() > endCollExcludeRadius_ &&
            (pos - endExcludeBall_).norm() > endCollExcludeRadius_ &&
            smoothedL1(collBallRadius_ - sdf, mu_, f, df))
        {
            pena += weight_ * f;
            return true;
        }
        return false;
    }
};

/**
 * @brief Collision Penalty for configuration space (config space)
 *
 */
class StompLegCollisionPenalty
{
private:
    std::shared_ptr<BaseRobotInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    Eigen::Vector3d collBallRadius_;
    Eigen::Vector3d weight_;
    double mu_;

    double endCollExcludeRadius_; // TODO: use gaussian weight better
    double endCollExcludeSmooth_;
    Eigen::Vector3d startExcludeBall_;
    Eigen::Vector3d endExcludeBall_;

    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    StompLegCollisionPenalty(std::shared_ptr<BaseRobotInterface> robot_interface,
                             std::shared_ptr<GridMapInterface> gridmap_interface,
                             std::shared_ptr<GCSVisualizer> visualizer = nullptr)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface)
    {
        setupVis(visualizer);
    }

    void setupVis(std::shared_ptr<GCSVisualizer> visualizer)
    {
        if (visualizer != nullptr)
        {
            enable_vis_ = true;
            visualizer_ = visualizer;
        }
    }

    void setupParams(SwingTrajPlannerConfig &config)
    {
        collBallRadius_ << config.CollBall1Rad, config.CollBall2Rad, config.CollBall3Rad;
        weight_ << config.CollBall1Weight, config.CollBall2Weight, config.CollBall3Weight;
        mu_ = config.smoothingFactor;
        endCollExcludeRadius_ = config.FootCollExcludeBallRad;
        endCollExcludeSmooth_ = config.FootCollExcludeBallSmoothRad;
    }

    /**
     * @brief Setup excluding ball for collision checking (cartesian space)
     *
     * @param start
     * @param end
     */
    void setExcludeBall(const Eigen::Vector3d start, const Eigen::Vector3d end)
    {
        startExcludeBall_ = start;
        endExcludeBall_ = end;
    }

    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }
    /**
     * @brief Attach penalty to position
     *
     * @param pose Pose of base
     * @param posCfg Position in config space
     * @param index Foot index
     * @param pena Penalty
     * @return if collision happens
     */
    bool attachPena(const pinocchio::SE3 &pose,
                    const Eigen::Vector3d &posCfg,
                    int index,
                    double &pena)
    {
        bool collFlag = false;
        Eigen::Matrix3Xd J;
        Eigen::Vector3d pos; // WORLD frame
        double f, df, sdf;
        std::string sdf_mode = "min"; // or min if celing exists

        // Foot Collision
        pos = point_SE3Act(pose.inverse(), robot_interface_->FK_foot(posCfg, index));
        sdf = gridmap_interface_->sdfValue(pos, sdf_mode);
        double start_dis = (pos - startExcludeBall_).norm();
        double end_dis = (pos - endExcludeBall_).norm();
        double exclude_weight = 1;
        if (start_dis > endCollExcludeRadius_ &&
            end_dis > endCollExcludeRadius_ &&
            smoothedL1(collBallRadius_(2) - sdf, mu_, f, df))
        {
            if (std::min(start_dis, end_dis) < endCollExcludeRadius_ + endCollExcludeSmooth_)
            {
                exclude_weight = sine_remap((std::min(start_dis, end_dis) - endCollExcludeRadius_) / endCollExcludeSmooth_);
                f *= exclude_weight;
            }
            pena += weight_(2) * f;
            collFlag = true;
            if (enable_vis_)
            {
                visualizer_->setIdGroup(3);
                visualizer_->visSphere(pos, collBallRadius_(2), ros_visualizer::VisStyle(0.5, 0.1, 0.1, 0.3, 0.005));
            }
        }

        // Joint2 Collision
        pos = point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(posCfg, index, 2));
        sdf = gridmap_interface_->sdfValue(pos, sdf_mode);
        if (smoothedL1(collBallRadius_(1) - sdf, mu_, f, df))
        {
            pena += weight_(1) * f;
            collFlag = true;
            if (enable_vis_)
            {
                visualizer_->setIdGroup(3);
                visualizer_->visSphere(point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(posCfg, index, 1)), collBallRadius_(0),
                                       ros_visualizer::VisStyle(0.8, 0.1, 0.1, 0.3, 0.005));
                visualizer_->visSphere(point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(posCfg, index, 2)), collBallRadius_(1),
                                       ros_visualizer::VisStyle(0.8, 0.1, 0.1, 0.3, 0.005));
                visualizer_->visSphere(point_SE3Act(pose.inverse(), robot_interface_->FK_foot(posCfg, index)), collBallRadius_(2),
                                       ros_visualizer::VisStyle(0.8, 0.1, 0.1, 0.3, 0.005));
            }
        }

        if (enable_vis_ && !collFlag)
        {
            visualizer_->setIdGroup(3);
            // visualizer_->visSphere(point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(posCfg, index, 1)), collBallRadius_(0), ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.1, 0.005));
            visualizer_->visSphere(point_SE3Act(pose.inverse(), robot_interface_->FK_foot(posCfg, index)), collBallRadius_(2), ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.18, 0.005));
        }

        return collFlag;
    }

    void visClear()
    {
        if (enable_vis_)
            visualizer_->delGroup(3);
    }
};