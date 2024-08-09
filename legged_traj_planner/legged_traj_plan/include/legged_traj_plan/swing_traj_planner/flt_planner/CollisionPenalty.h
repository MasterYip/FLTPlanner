/**
 * @file CollisionPenalty.h
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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/utils/Geometry.h"

#include "Utils.h"

/**
 * @brief Collision Penalty for cartesian space (foot)
 *
 */
class CollisionPenalty
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
    CollisionPenalty(std::shared_ptr<GridMapInterface> gridmap_interface,
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
     * @param gradPos Gradient of position in world frame
     * @param pena Penalty
     */
    void attachPena(const Eigen::Vector3d &pos,
                    Eigen::Vector3d &gradPos,
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
            sdfGrad = gridmap_interface_->sdfDerivative(pos, 0);
            gradPos += -df * sdfGrad / sdfGrad.norm();
            pena += weight_ * f;
            if (enable_vis_)
            {
                visualizer_->setIdGroup(3);
                visualizer_->visArrow(pos, pos + gradPos * 0.1, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.3, 0.005));
            }
        }
    }
};

/**
 * @brief Collision Penalty for configuration space (config space)
 *
 */
class LegCollisionPenalty
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    Eigen::Vector3d collBallRadius_;
    Eigen::Vector3d weight_;
    double mu_;

    double endCollExcludeRadius_; // TODO: use gaussian weight better
    Eigen::Vector3d startExcludeBall_;
    Eigen::Vector3d endExcludeBall_;

    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    LegCollisionPenalty(std::shared_ptr<ElSpiderAirInterface> robot_interface,
                        std::shared_ptr<GridMapInterface> gridmap_interface,
                        std::shared_ptr<GCSVisualizer> visualizer = nullptr)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface)
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
        collBallRadius_ << config.CollBall1Rad, config.CollBall2Rad, config.CollBall3Rad;
        weight_ << config.CollBall1Weight, config.CollBall2Weight, config.CollBall3Weight;
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
     *
     * @param pose Pose of base
     * @param posCfg Position in config space
     * @param gradPosCfg Gradient of position in config space
     * @param pena Penalty
     */
    void attachPena(const pinocchio::SE3 &pose,
                    const Eigen::Vector3d &posCfg,
                    const Eigen::Vector3d &velCfg,
                    const Eigen::Vector3d &accCfg,
                    Eigen::Vector3d &gradPosCfg,
                    int index,
                    double &pena)
    {
        Eigen::Matrix3Xd J, dJ;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos, vel, acc, kappa, veldir, sdfGrad, gradPcoll; // WORLD frame
        double f, df, velnorm, sdf;
        std::string sdf_mode = "min"; // or min if celing exists

        // Foot Collision
        pos = point_SE3Act(pose.inverse(), robot_interface_->FK_foot(posCfg, index));
        sdf = gridmap_interface_->sdfValue(pos, sdf_mode);
        if ((pos - startExcludeBall_).norm() > endCollExcludeRadius_ &&
            (pos - endExcludeBall_).norm() > endCollExcludeRadius_ &&
            smoothedL1(collBallRadius_(2) - sdf, mu_, f, df))
        {
            J = robot_interface_->getJacobian(posCfg, index);
            dJ = robot_interface_->getJacobianTimeVariation(posCfg, velCfg, index);
            vel = vec_SE3Act(pose.inverse(), J * velCfg);
            acc = vec_SE3Act(pose.inverse(), J * accCfg + dJ * velCfg);
            veldir = vel;
            veldir.normalize();
            velnorm = vel.norm();
            kappa = 1 / (velnorm * velnorm) * (I - veldir * veldir.transpose()) * acc;
            sdfGrad = gridmap_interface_->sdfDerivative(pos, 0);
            gradPcoll = -df * sdfGrad / sdfGrad.norm();
            Eigen::Vector3d dg = weight_(2) * velnorm * J.transpose() *
                                 ((I - veldir * veldir.transpose()) * gradPcoll - f * kappa);
            // FIXME: avoid Nan
            // NaN check
            if (isnan(dg(0)) || isnan(dg(1)) || isnan(dg(2)))
                std::cout << "Warning: dg is " << dg.transpose() << std::endl;
            else
                gradPosCfg += dg;
            pena += weight_(2) * f * velnorm;
            if (enable_vis_)
            {
                visualizer_->setIdGroup(3);
                visualizer_->visArrow(pos, pos + gradPcoll * 0.1, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.3, 0.005));
                visualizer_->visArrow(posCfg, posCfg + gradPosCfg * 0.1, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.3, 0.005));
            }
        }

        // Joint2 Collision
        pos = point_SE3Act(pose.inverse(), robot_interface_->FK_CollBall(posCfg, index, 2));
        sdf = gridmap_interface_->sdfValue(pos, sdf_mode);
        if (smoothedL1(collBallRadius_(1) - sdf, mu_, f, df))
        {
            J = robot_interface_->getJacobian_CollBall(posCfg, index, 2);
            dJ = robot_interface_->getJacobianTimeVariation_CollBall(posCfg, velCfg, index, 2);
            vel = vec_SE3Act(pose.inverse(), J * velCfg);
            acc = vec_SE3Act(pose.inverse(), J * accCfg + dJ * velCfg);
            veldir = vel;
            veldir.normalize();
            velnorm = vel.norm();
            kappa = 1 / (velnorm * velnorm) * (I - veldir * veldir.transpose()) * acc;
            sdfGrad = gridmap_interface_->sdfDerivative(pos, 0);
            gradPcoll = -df * sdfGrad / sdfGrad.norm();
            Eigen::Vector3d dg = weight_(1) * velnorm * J.transpose() *
                                 ((I - veldir * veldir.transpose()) * gradPcoll - f * kappa);
            // NaN check
            if (isnan(dg(0)) || isnan(dg(1)) || isnan(dg(2)))
                std::cout << "Warning: dg is " << dg.transpose() << std::endl;
            else
                gradPosCfg += dg;
            pena += weight_(1) * f * velnorm;
            if (enable_vis_)
            {
                visualizer_->setIdGroup(3);
                visualizer_->visArrow(pos, pos + gradPcoll * 0.4, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.3, 0.005));
            }
        }
    }

    void visClear()
    {
        if (enable_vis_)
            visualizer_->delGroup(3);
    }
};