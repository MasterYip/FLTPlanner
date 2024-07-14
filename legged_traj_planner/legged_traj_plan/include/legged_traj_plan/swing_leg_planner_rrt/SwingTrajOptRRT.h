/**
 * @file SwingTrajOptRRT.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-07-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
/* external project header files */
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// class RRT_SearchSpace
// {
// };

class SwingTrajOptRRT
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    // RRT
    std::shared_ptr<ob::RealVectorStateSpace> space_;

    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    SwingTrajOptRRT(std::shared_ptr<ElSpiderAirInterface> robot_interface,
                    std::shared_ptr<GridMapInterface> gridmap_interface,
                    std::shared_ptr<GCSVisualizer> visualizer = nullptr,
                    bool enable_benchmark = true)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          visualizer_(visualizer),
          legCollPena(robot_interface, gridmap_interface), collPena(gridmap_interface_),
          benchmark_("SwingTrajOpt", enable_benchmark)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;
    };

    bool isStateValid(const ob::State *state)
    {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        // Define some simple obstacles (e.g., circular obstacles)
        std::vector<std::pair<double, double>> obstacles = {
            {1.0, 1.0},
            {2.0, 2.0}};
        double radius = 0.5;

        for (const auto &obstacle : obstacles)
        {
            double dist = std::sqrt(std::pow(pos->values[0] - obstacle.first, 2) +
                                    std::pow(pos->values[1] - obstacle.second, 2));
            if (dist <= radius)
                return false;
        }
        return true;
    }

    /**
     * @brief Setup MINCO optimization problem
     *
     * @param TrajPolyPath Config space poly path
     * @param initialVel Initial position, velocity
     * @param terminalVel Terminal position, velocity
     * @return true
     * @return false
     */
    inline bool setup(
        // Conditions
        const pinocchio::SE3 &pose0,
        const pinocchio::SE3 &pose1,
        const int &index,
        // Init waypoints
        // const Eigen::Matrix3Xd &TrajPolyPath,
        const Eigen::Vector3d &initialVel,
        const Eigen::Vector3d &terminalVel,
        // Params
        SwingTrajPlannerConfig &config,
        // Settings
        const bool verbose = true) {
    };