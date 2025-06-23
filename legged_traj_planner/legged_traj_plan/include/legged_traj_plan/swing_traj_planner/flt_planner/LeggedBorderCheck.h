/**
 * @file LeggedBorderCheck.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/poly_traj/border_check.hpp"
/* external project header files */

struct LeggedBorderCheckConfig
{
    std::string ground_layer = "elevation";
    std::string ceiling_layer = "ceiling";
    bool enable_ground = true;
    bool enable_ceiling = false;

    int guide_surf_type = 1;
    // 0: grid map ground (not using guide surf)
    // 1: harmonic guide surf
    // 2: convoluted guide surf
    int guid_surf_conv_samples = 3;
    double guid_surf_conv_interval = 0.1;

    int interp_mode = 1;
    // 0: progress =  <(p1 - p0), (p - p0)> / |p1 - p0|
    // 1: progress =  <(nominal_pos1 - nominal_pos0), (p - nominal_pos0)> / |nominal_pos1 - nominal_pos0|

    double collBallRad1 = 0.0;
    double collBallRad2 = 0.0; // Knee
    double collBallRad3 = 0.0; // Foot
    double FootCollExcludeBallRad = 0.0;
};

class LeggedBorderCheck : public BorderCheckBase
{
private:
    const grid_map::GridMap &map_;
    IndexRemap index_remap_;
    std::unique_ptr<BaseGuideSurf> guide_surf_ptr_;
    std::shared_ptr<BaseRobotInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    LeggedBorderCheckConfig config_;

    pinocchio::SE3 pose0_;
    pinocchio::SE3 pose1_;
    Eigen::Vector3d nominal_joint_pos_{0, 1, 1};
    Eigen::Vector3d nominal_pos0_;
    Eigen::Vector3d nominal_pos1_;
    Eigen::Vector3d p0_;
    Eigen::Vector3d p1_;
    int index_;

public:
    LeggedBorderCheck(std::shared_ptr<BaseRobotInterface> robot_interface,
                      std::shared_ptr<GridMapInterface> gridmap_interface,
                      const pinocchio::SE3 &pose0,
                      const pinocchio::SE3 &pose1,
                      const Eigen::Vector3d &p0,
                      const Eigen::Vector3d &p1,
                      const int &index,
                      const LeggedBorderCheckConfig &config = LeggedBorderCheckConfig())
        : map_(gridmap_interface->getMap()),
          index_remap_(map_),
          robot_interface_(robot_interface),
          gridmap_interface_(gridmap_interface),
          config_(config), pose0_(pose0), pose1_(pose1),
          p0_(p0), p1_(p1), index_(index)
    {
        if (config_.guide_surf_type == 0)
        {
            // Grid Map Ground
            guide_surf_ptr_ = std::make_unique<BaseGuideSurf>();
        }
        else if (config_.guide_surf_type == 1)
        {
            // Guide Surf
            int samples = 3;
            Eigen::Vector3d pmid = (p0_ + p1_) / 2;
            double h = 0;

            // Ave
            // for (int i = 1; i < samples + 1; i++)
            // {
            //     h += gridmap_interface_->value(
            //         (p0_ + (p1_ - p0_) * i / (samples + 1)).head(2),
            //         config_.ground_layer);
            // }
            // pmid[2] = h / samples;

            // Max
            for (int i = 1; i < samples + 1; i++)
            {
                pmid[2] = std::max(pmid[2], gridmap_interface_->value(
                                                (p0_ + (p1_ - p0_) * i / (samples + 1)).head(2),
                                                config_.ground_layer));
            }
            std::vector<Point3D> key_points = {p0_, pmid, p1_};
            guide_surf_ptr_ = std::make_unique<HarmonicGuideSurf>(key_points);
        }
        else if (config_.guide_surf_type == 2)
        {
            // Guide Surf
            guide_surf_ptr_ = std::make_unique<ConvolutedGuideSurf>(map_, config_.guid_surf_conv_samples,
                                                                    config_.guid_surf_conv_interval, config_.ground_layer);
        }

        // Nominal Pos
        if (config_.interp_mode == 0)
        {
            nominal_pos0_ = p0_;
            nominal_pos1_ = p1_;
        }
        else if (config_.interp_mode == 1)
        {
            nominal_pos0_ = point_SE3Act(pose0_.inverse(), robot_interface_->FK_foot(nominal_joint_pos_, index_));
            nominal_pos1_ = point_SE3Act(pose1_.inverse(), robot_interface_->FK_foot(nominal_joint_pos_, index_));
        }
    }

    double projectInterp(const Eigen::Vector2d &pos2d);

    // Override interfaces
    double queryHeight(const Eigen::Vector2d &pos2d) override;

    double queryHeight(const GridPt &grid2d) override;

    double disInBorder(const Eigen::Vector2d &pos2d) override;

    double disInBorder(const GridPt &grid2d) override;

    bool isStartValid(const GridPt &start) override;

    bool isGoalValid(const GridPt &goal) override;

    bool isStartValid(const Eigen::Vector2d &start) override;

    bool isGoalValid(const Eigen::Vector2d &goal) override;
};
