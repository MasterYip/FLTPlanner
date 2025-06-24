/**
 * @file UnitreeA1PlannerBase.h
 * @author GitHub Copilot
 * @brief Unitree A1 Planner Base - Common functionality for A1 planners
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */

#include "legged_traj_plan/robot_interface/UnitreeA1Interface.h" // Should be included first (pinocchio)
#include "legged_traj_plan/robot_interface/DummyUnitreeA1InterfaceROS.h"

#include "legged_traj_plan/perception_interface/GridMapInterface.h"

#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_plan/swing_traj_planner/simple_planner/SimplePlanner.h"
#include "legged_traj_plan/swing_traj_planner/fec_planner/FECPlanner.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/rrt_planner/RRTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/stomp_planner/StompPlanner.h"

#include "legged_traj_search/utils/gcs_visualizer.hpp"

/* external project header files */
#include <ros/ros.h>

// BUG
// IMPORTANT: Add this function to avoid Convex hull display error. (unknown reason)
void AVOID_DISPLAY_ERROR_A1(void)
{
    Eigen::Vector3d vec(1, 1, 1);
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vec.data(), vec.cols(), false, false);
}

class UnitreeA1PlannerBase
{
protected:
    ros::NodeHandle nh_;

    // Interface
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::string robot_interface_type_;
    std::shared_ptr<UnitreeA1Interface> robot_interface_;
    std::shared_ptr<UnitreeA1Interface> robot_interface_shadow_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;

    // Config
    SwingTrajPlannerConfig swing_traj_planner_config_;

public:
    UnitreeA1PlannerBase() : nh_("~")
    {
        // Robot Interface
        nh_.getParam("robotInterface", robot_interface_type_);
        if (robot_interface_type_ == "UnitreeA1Dummy")
        {
            DummyUnitreeA1InterfaceROSConfig dummy_config;
            dummy_config.loadParam(nh_, "UnitreeA1Dummy");
            robot_interface_ = std::make_shared<DummyUnitreeA1InterfaceROS>(dummy_config);
        }
        else if (robot_interface_type_ == "UnitreeA1ROS")
        {
            // For real robot interface, use URDF from parameter server
            std::string urdf_path = nh_.param<std::string>("/robot_description", "");
            robot_interface_ = std::make_shared<UnitreeA1Interface>(urdf_path);
        }
        else
        {
            ROS_ERROR("Unknown robot interface type: %s", robot_interface_type_.c_str());
            // Default fallback
            std::string urdf_path = nh_.param<std::string>("/robot_description", "");
            robot_interface_ = std::make_shared<UnitreeA1Interface>(urdf_path);
        }

        // Shadow Interface (for visualization)
        DummyUnitreeA1InterfaceROSConfig interface_shadow_config;
        interface_shadow_config.loadParam(nh_, "UnitreeA1Shadow");
        robot_interface_shadow_ = std::make_shared<DummyUnitreeA1InterfaceROS>(interface_shadow_config);

        // GridMap Interface
        GridMapInterfaceConfig gridmap_interface_config;
        gridmap_interface_config.loadParam(nh_);
        gridmap_interface_ = std::make_shared<GridMapInterface>(nh_, gridmap_interface_config);

        // Swing Traj Planner
        swing_traj_planner_config_.loadParams(nh_);
        if (swing_traj_planner_config_.plannerID == 0)
        {
            if (swing_traj_planner_config_.useCfgSpace)
            {
                swing_traj_planner_ = std::make_shared<FLTCfgPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
            }
            else
            {
                swing_traj_planner_ = std::make_shared<FLTPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
            }
        }
        else if (swing_traj_planner_config_.plannerID == 1)
        {
            swing_traj_planner_ = std::make_shared<RRTPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else if (swing_traj_planner_config_.plannerID == 2)
        {
            swing_traj_planner_ = std::make_shared<RRTCfgPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else if (swing_traj_planner_config_.plannerID == 3)
        {
            swing_traj_planner_ = std::make_shared<HeightClearPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else if (swing_traj_planner_config_.plannerID == 4)
        {
            swing_traj_planner_ = std::make_shared<StompPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else if (swing_traj_planner_config_.plannerID == 5)
        {
            swing_traj_planner_ = std::make_shared<StompCfgPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else if (swing_traj_planner_config_.plannerID == 6)
        {
            swing_traj_planner_ = std::make_shared<FECPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
        else
        {
            ROS_ERROR("Invalid planner ID");
            // Default to FLT planner
            swing_traj_planner_ = std::make_shared<FLTCfgPlanner>(swing_traj_planner_config_, robot_interface_, gridmap_interface_);
        }
    }
};