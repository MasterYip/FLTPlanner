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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/config.h>

/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief Check if a point is in the exclude cylinder
 *
 * @note The exclude cylinder is defined by a center and a radius,
 * the top height is infinite, the bottom height is at the center height minus the radius
 * @param pos
 * @param center
 * @param radius
 * @return true
 * @return false
 */
inline bool inExcludeCylinder(const Eigen::Vector3d &pos, const Eigen::Vector3d &center, double radius)
{
    return (pos.head(2) - center.head(2)).norm() < radius && pos(2) > center(2) - radius;
}

class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr &si) : ob::StateCostIntegralObjective(si, true)
    {
    }

    ob::Cost stateCost(const ob::State *s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

class SwingTrajOptRRT
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    // RRT
    double collball_radius_;
    double exclude_radius_;
    double coll_margin_;

    Eigen::Vector3d start_exclude_cylinder_;
    Eigen::Vector3d end_exclude_cylinder_;
    std::shared_ptr<ob::RealVectorStateSpace> space_;

    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

    // Benchmarking
    Benchmark benchmark_;

public:
    SwingTrajOptRRT(SwingTrajPlannerConfig config,
                    std::shared_ptr<ElSpiderAirInterface> robot_interface,
                    std::shared_ptr<GridMapInterface> gridmap_interface,
                    std::shared_ptr<GCSVisualizer> visualizer = nullptr,
                    bool enable_benchmark = true)
        : config_(config), robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          space_(std::make_shared<ob::RealVectorStateSpace>(3)),
          visualizer_(visualizer),
          benchmark_("SwingTrajOptRRT", enable_benchmark)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;
        setupParams(config);
    };

    void setupParams(SwingTrajPlannerConfig &config)
    {
        collball_radius_ = config.collBallRadius;
        exclude_radius_ = config.excludeRadius;
        coll_margin_ = config.collMargin;
    }

    bool isStateValid(const ob::State *state)
    {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d pos_vec(pos->values[0], pos->values[1], pos->values[2]);
        double sdf = gridmap_interface_->sdfValue(pos_vec, "min");
        if (!inExcludeCylinder(pos_vec, start_exclude_cylinder_, exclude_radius_) &&
            !inExcludeCylinder(pos_vec, end_exclude_cylinder_, exclude_radius_) &&
            collball_radius_ > sdf - coll_margin_)
            return false;
        return true;
    }

    ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
        ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

        return 10.0 * lengthObj + 0.1 * clearObj;
    }

    inline bool optimize(UniBSpline &traj, SwingTrajPlannerConfig &config, double max_time = 0.1)
    {
        // Setup Params
        start_exclude_cylinder_ = traj.evaluate(0, 0, true);
        end_exclude_cylinder_ = traj.evaluate(1, 0, true);

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

        // Optimization objective
        ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation()));

        // Create an RRT* planner
        // auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        // auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation())); // FIXME: error
        auto planner(std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation()));
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