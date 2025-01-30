/**
 * @file FECPlanner.h
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
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_plan/swing_traj_planner/fec_planner/FECCheck.h"

class FECPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    FECCheck fec_check_;

public:
    FECPlanner(SwingTrajPlannerConfig config,
               std::shared_ptr<ElSpiderAirInterface> robot_interface,
               std::shared_ptr<GridMapInterface> gridmap_interface)
        : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
          fec_check_(robot_interface, gridmap_interface)
    {
        FECCheckConfig fec_check_config;
        fec_check_config.collBallRad2 = config.collBallCheckRad2;
        fec_check_config.collBallRad3 = config.collBallCheckRad3;
        fec_check_config.FootCollExcludeBallRad = config.FootCollExcludeBallRad;
        fec_check_config.checkResolution = config.FECCheckResolution;
        fec_check_.setConfig(fec_check_config);
        visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    }

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index) override
    {
        double h_lift = config_.hLift;
        double v_lift = config_.vLift;
        Eigen::MatrixXd knots(6, 3);
        // Start
        knots.row(0) = p0;
        Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
        normal.normalize();
        knots.row(1) = normal * v_lift;

        int samples = 20;
        Eigen::Vector3d pmid = (p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift);

        for (int i = 0; i < samples; i++)
        {
            double t = i / (samples - 1.0);
            Eigen::Vector2d p = p0.head(2) * (1 - t) + p1.head(2) * t;
            if (gridmap_interface_->value(p) > pmid[2])
            {
                pmid = Eigen::Vector3d(p[0], p[1], gridmap_interface_->value(p));
            }
        }
        knots.row(2) = pmid;
        Eigen::Vector3d vmid = p1 - p0;
        vmid = vmid.normalized() * v_lift;
        knots.row(3) = vmid;
        // Goal
        knots.row(4) = p1;
        normal = gridmap_interface_->sdfDerivative(p1, 0);
        normal.normalize();
        knots.row(5) = -normal * v_lift;

        return std::make_shared<CubicHermiteSpline>(knots);
    }

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index) override
    {
        if (config_.enableVis)
        {
            std::vector<Point3D> path;
            double ts = 0.01;
            double t = 0;
            while (t < 1.0)
            {
                path.emplace_back(traj->evaluate(t, 0, true));
                t += ts;
            }
            visualizer_->setIdGroup(1);
            visualizer_->visCurve(path, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
        }
        return true;
    }

    bool reachableCheckHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                            Eigen::Vector3d p0, uint index,
                            std::vector<Eigen::Vector3d> &footholds,
                            std::vector<bool> &reachable) override
    {
        for (size_t i = 0; i < footholds.size(); i++)
        {
            auto traj = getInitTrajHook(pose0, pose1, p0, footholds[i], index);
            reachable[i] = fec_check_.checkTrajReachability(pose0, pose1, traj, p0, footholds[i], index);
        }
        return true;
    };
};