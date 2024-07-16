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
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
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

    // Benchmarking
    Benchmark benchmark_;

public:
    SwingTrajOptRRT(std::shared_ptr<ElSpiderAirInterface> robot_interface,
                    std::shared_ptr<GridMapInterface> gridmap_interface,
                    std::shared_ptr<GCSVisualizer> visualizer = nullptr,
                    bool enable_benchmark = true)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          space_(std::make_shared<ob::RealVectorStateSpace>(3)),
          visualizer_(visualizer),
          benchmark_("SwingTrajOptRRT", enable_benchmark)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;
    };

    bool isStateValid(const ob::State *state)
    {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        // Define some simple obstacles (e.g., circular obstacles)
        
        return true;
    }

    // /**
    //  * @brief Setup RRTStar optimization problem
    //  *
    //  * @return true
    //  * @return false
    //  */
    // inline bool setup(
    //     // Conditions
    //     const int &index,
    //     // Params
    //     SwingTrajPlannerConfig &config,
    //     // Settings
    //     const bool verbose = true) {
    //     ob::RealVectorBounds bounds(3);
    //     // bounds[0].setLow(-1);
    // };

    inline bool optimize(UniBSpline &traj, SwingTrajPlannerConfig &config, double max_time = 0.1)
    {
        // Set Bounds
        ob::RealVectorBounds bounds(3);
        Eigen::MatrixXd knots = traj.get();
        double margin = 0.6; // Margin of the bounding box
        for (int i = 0; i < 3; i++)
        {
            bounds.setLow(i, knots.col(i).minCoeff() - margin);
            bounds.setHigh(i, knots.col(i).maxCoeff() + margin);
        }
        space_->setBounds(bounds);

        // Create a SimpleSetup object
        og::SimpleSetup ss(space_);

        // Set state validity checking for this space
        ss.setStateValidityChecker(std::bind(&SwingTrajOptRRT::isStateValid, this, std::placeholders::_1));

        // Define start and goal states
        ob::ScopedState<> start(space_);
        start[0] = knots(0, 0);
        start[1] = knots(0, 1);
        start[2] = knots(0, 2);

        ob::ScopedState<> goal(space_);
        goal[0] = knots(knots.rows() - 1, 0);
        goal[1] = knots(knots.rows() - 1, 1);
        goal[2] = knots(knots.rows() - 1, 2);

        // Set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // Create an RRT* planner
        auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        ss.setPlanner(planner);

        // Attempt to solve the problem within a given time (seconds)
        ob::PlannerStatus solved = ss.solve(max_time);

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            ss.simplifySolution();
            ss.getSolutionPath().printAsMatrix(std::cout);
            Eigen::MatrixXd new_knots(ss.getSolutionPath().getStateCount(), 3);
            for (std::size_t i = 0; i < ss.getSolutionPath().getStateCount(); ++i)
            {
                const auto *pos = ss.getSolutionPath().getState(i)->as<ob::RealVectorStateSpace::StateType>();
                new_knots.row(i) << pos->values[0], pos->values[1], pos->values[2];
            }
            traj.set(new_knots);
            return true;
        }
        else
        {
            std::cout << "No solution found" << std::endl;
            return false;
        }
    }
};