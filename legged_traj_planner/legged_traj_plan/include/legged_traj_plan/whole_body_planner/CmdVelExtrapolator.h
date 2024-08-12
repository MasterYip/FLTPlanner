/**
 * @file CmdVelExtrapolator.h
 * @author Master Yip (2205029492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <memory>
/* external project header files */
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <geometry_msgs/Twist.h>

/* internal project header files */
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"


using PosList = std::vector<Eigen::Vector3d>;

class CmdVelExtrapolator
{
protected:
    geometry_msgs::Twist cmd_vel_; // cmd vel relative to the BASE frame
    pinocchio::SE3 pose_;

public:
    CmdVelExtrapolator() = default;
    void update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel = geometry_msgs::Twist())
    {
        // FIXME: is default cmd_vel 0?
        cmd_vel_ = cmd_vel;
        pose_ = pose;
    }

    /**
     * @brief Extrapolate the pose by dt
     *
     * @param dt Delta t
     * @return pinocchio::SE3
     */
    virtual pinocchio::SE3 extrapolate(double dt)
    {
        pinocchio::SE3 pose_new = pose_;
        Eigen::Vector3d linear_world;
        linear_world << cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z;
        linear_world = pose_.rotation() * linear_world;
        Eigen::Vector3d angular_world;
        angular_world << cmd_vel_.angular.x, cmd_vel_.angular.y, cmd_vel_.angular.z;
        angular_world = pose_.rotation() * angular_world;
        pinocchio::Motion angular_world_motion;
        angular_world_motion.linear() = Eigen::Vector3d::Zero();
        angular_world_motion.angular() = angular_world;

        pose_new.translation() += linear_world * dt;
        pose_new.rotation() = pose_.rotation() * pinocchio::exp6(angular_world_motion * dt).rotation();
        return pose_new;
    }
};

class GridMapCmdVelExtrapolator : public CmdVelExtrapolator
{
protected:
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    PosList exp_pose_samples_; // Expected pose sample points in base frame
    double nominal_height_;

public:
    void init(std::shared_ptr<GridMapInterface> gridmap_interface,
              PosList exp_pose_samples, double nominal_height = 0.25)
    {
        gridmap_interface_ = gridmap_interface;
        exp_pose_samples_ = exp_pose_samples;
        nominal_height_ = nominal_height;
    }

    pinocchio::SE3 extrapolate(double dt) override
    {
        PosList map_sample_projection;
        pinocchio::SE3 pose_new = CmdVelExtrapolator::extrapolate(dt);
        // FIXME: performance can be improved
        for (auto &pos_base : exp_pose_samples_)
        {
            auto pos_world = point_SE3Act(pose_new.inverse(), pos_base);
            pos_world(2) = gridmap_interface_->value(grid_map::Position(pos_world(0), pos_world(1)));
            map_sample_projection.emplace_back(pos_world);
        }

        // plane fitting
        Eigen::Vector3d plane;
        plane_fitting(map_sample_projection, plane);
        pose_new.translation()[2] = plane(0) * pose_new.translation()[0] + plane(1) * pose_new.translation()[1] + plane(2) + nominal_height_;
        pinocchio::Motion rot_calib_motion;
        Eigen::Vector3d body_normal = pose_.rotation().col(2);
        Eigen::Vector3d exp_normal = Eigen::Vector3d(-plane(0), -plane(1), 1);
        exp_normal.normalize();
        rot_calib_motion.angular() = body_normal.cross(exp_normal);
        pose_new.rotation() = pinocchio::exp6(rot_calib_motion).rotation() * pose_new.rotation();
        return pose_new;
    }
};
