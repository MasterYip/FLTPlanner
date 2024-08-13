/**
 * @file LeggedBorderCheck.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/swing_traj_planner/flt_planner/LeggedBorderCheck.h"

double LeggedBorderCheck::queryHeight(const Eigen::Vector2d &pos2d)
{
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (enable_ground_ && query_height < map_.atPosition(config_.ground_layer, pos2d))
    {
        query_height = map_.atPosition(config_.ground_layer, pos2d);
    }
    if (enable_ceiling_ && query_height > map_.atPosition(config_.ceiling_layer, pos2d))
    {
        query_height = map_.atPosition(config_.ceiling_layer, pos2d);
    }
    return query_height;
};

double LeggedBorderCheck::queryHeight(const GridPt &grid2d)
{
    Eigen::Vector2d pos2d;
    GridPt index = index_remap_.grid2Index(grid2d);
    map_.getPosition(index, pos2d);
    geo_utils_2d::Point pos;
    pos << pos2d(0), pos2d(1);
    double query_height = poly_corridor_.getGuideSurf().getHeight(pos);
    if (enable_ground_ && query_height < map_.at(config_.ground_layer, index))
    {
        query_height = map_.at(config_.ground_layer, index);
    }
    if (enable_ceiling_ && query_height > map_.at(config_.ceiling_layer, index))
    {
        query_height = map_.at(config_.ceiling_layer, index);
    }
    return query_height;
};

double LeggedBorderCheck::projectInterp(const Eigen::Vector2d &pos2d)
{
    Eigen::Vector2d vec = p1_ - p0_;
    double len = vec.norm();
    vec.normalize();
    double len_proj = (pos2d - p0_).dot(vec);   
    return std::clamp(len_proj, 0.0, len) / len;
}

// FIXME: this function can't return distance in border, only return whether in border
double LeggedBorderCheck::disInBorder(const Eigen::Vector2d &pos2d)
{
    pinocchio::SE3 pose = poseLinearInterp(pose0_, pose1_, projectInterp(pos2d));
    Eigen::Vector3d pos(pos2d(0), pos2d(1), queryHeight(pos2d));
    pos = point_SE3Act(pose, pos);
    Eigen::Vecto3d sol;
    return robot_interface_->getRobotKin().inverseKinConstraint(pos, sol, index_, false) ? 1.0 : -1.0;
}

double LeggedBorderCheck::disInBorder(const GridPt &grid2d)
{
    return disInBorder(index_remap_.grid2Pos(grid2d));
}

bool LeggedBorderCheck::isStartValid(const GridPt &start)
{
    return disInBorder(start) > 0.0;
}

bool LeggedBorderCheck::isGoalValid(const GridPt &goal)
{
    return disInBorder(goal) > 0.0;
}