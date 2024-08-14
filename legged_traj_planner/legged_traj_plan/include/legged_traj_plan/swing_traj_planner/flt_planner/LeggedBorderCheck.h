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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/poly_traj/border_check.hpp"
/* external project header files */

struct LeggedBorderCheckConfig
{
    std::string ground_layer = "elevation";
    std::string ceiling_layer = "ceiling";
    bool enable_ground = true;
    bool enable_ceiling = false;

    double collBallRad1 = 0.0;
    double collBallRad2 = 0.0;
    double collBallRad3 = 0.0;
};

class LeggedBorderCheck : public BorderCheckBase
{
private:
    const grid_map::GridMap &map_;
    IndexRemap index_remap_;
    HarmonicGuideSurf guide_surf_;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    LeggedBorderCheckConfig config_;

    pinocchio::SE3 pose0_;
    pinocchio::SE3 pose1_;
    Eigen::Vector3d p0_;
    Eigen::Vector3d p1_;
    int index_;

public:
    LeggedBorderCheck(std::shared_ptr<ElSpiderAirInterface> robot_interface,
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
        int samples = 3;
        Eigen::Vector3d pmid = (p0_ + p1_) / 2;
        double h = 0;
        for (int i = 1; i < samples + 1; i++)
        {
            h += gridmap_interface_->value(
                (p0_ + (p1_ - p0_) * i / (samples + 1)).head(2),
                config_.ground_layer);
        }
        pmid[2] = h / samples;
        std::vector<Point3D> key_points = {p0_, pmid, p1_};
        guide_surf_ = HarmonicGuideSurf(key_points);
    }

    double projectInterp(const Eigen::Vector2d &pos2d);
    // override
    double queryHeight(const Eigen::Vector2d &pos2d) override;

    double queryHeight(const GridPt &grid2d) override;

    double disInBorder(const Eigen::Vector2d &pos2d) override;

    double disInBorder(const GridPt &grid2d) override;

    bool isStartValid(const GridPt &start) override;

    bool isGoalValid(const GridPt &goal) override;
};
