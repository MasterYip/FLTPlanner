/**
 * @file ElSpiderAirPlannerBase.h
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

/* internal project header files */

#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h" // Should be included first (pinocchio)
#include "legged_traj_plan/robot_interface/DummyElSpiderAirInterfaceROS.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROSVMC.h"
#include "legged_traj_plan/robot_interface/DummyHexapod201InterfaceROS.h"

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
void AVOID_DISPLAY_ERROR(void)
{
    Eigen::Vector3d vec(1, 1, 1);
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vec.data(), vec.cols(), false, false);
}

class ElSpiderAirPlannerBase
{
protected:
    ros::NodeHandle nh_;

    // Interface
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::string robot_interface_type_;
    std::shared_ptr<BaseRobotInterface> robot_interface_;
    std::shared_ptr<BaseRobotInterface> robot_interface_shadow_;
    std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_;

    // Config
    SwingTrajPlannerConfig swing_traj_planner_config_;

public:
    ElSpiderAirPlannerBase() : nh_("~")
    {
        // Robot Interface
        nh_.getParam("robotInterface", robot_interface_type_);
        if (robot_interface_type_ == "ElSpiderAirDummy")
        {
            DummyElSpiderAirInterfaceROSConfig dummy_config;
            dummy_config.loadParam(nh_, "ElSpiderAirDummy");
            robot_interface_ = std::make_shared<DummyElSpiderAirInterfaceROS>(dummy_config);
        }
        else if (robot_interface_type_ == "ElSpiderAirROS")
        {
            ElSpiderAirInterfaceROSConfig config;
            config.loadParam(nh_, "ElSpiderAirROS");
            robot_interface_ = std::make_shared<ElSpiderAirInterfaceROS>(config);
        }
        else if (robot_interface_type_ == "ElSpiderAirROSVMC")
        {
            ElSpiderAirInterfaceROSVMCConfig config;
            config.loadParam(nh_, "ElSpiderAirROSVMC");
            robot_interface_ = std::make_shared<ElSpiderAirInterfaceROSVMC>(config);
        }
        else if (robot_interface_type_ == "Hexapod201Dummy")
        {
            DummyHexapod201InterfaceROSConfig dummy_config;
            dummy_config.loadParam(nh_, "Hexapod201Dummy");
            robot_interface_ = std::make_shared<DummyHexapod201InterfaceROS>(dummy_config);
        }
        else
        {
            ROS_ERROR("Unknown robot interface type: %s", robot_interface_type_.c_str());
        }

        // Shadow Interface
        DummyElSpiderAirInterfaceROSConfig interface_shadow_config;
        interface_shadow_config.loadParam(nh_, "ElSpiderAirShadow");
        robot_interface_shadow_ = std::make_shared<DummyElSpiderAirInterfaceROS>(interface_shadow_config);

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
        }
    }
};
