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
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Path.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/config.h>

/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
    int index_;

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
        if (!inZCylinder(pos_vec, start_exclude_cylinder_, exclude_radius_) &&
            !inZCylinder(pos_vec, end_exclude_cylinder_, exclude_radius_) &&
            collball_radius_ > sdf - coll_margin_)
            return false;
        // if (robot_interface_->getFootPolyhedra(index).)
        return true;
    }

    ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
        // ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

        // return 10.0 * lengthObj + 0.1 * clearObj;
        lengthObj->setCostThreshold(ob::Cost(10.0));
        return lengthObj;
    }

    inline bool optimize(std::shared_ptr<TrajectoryBase> &traj, int index)
    {
        // Setup Params
        index_ = index;
        start_exclude_cylinder_ = traj->evaluate(0, 0, true);
        end_exclude_cylinder_ = traj->evaluate(1, 0, true);
        double max_time = config_.maxTime;

        // Set Bounds
        ob::RealVectorBounds bounds(3);
        std::vector<Eigen::Vector3d> traj_points = std::dynamic_pointer_cast<MincoTrajectory>(traj)->getPolyPath();
        Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(traj_points.size(), 3);
        for (size_t i = 0; i < traj_points.size(); i++)
        {
            knots.row(i) = traj_points[i];
        }
        double margin = 0.1; // Margin of the bounding box
        for (int i = 0; i < 3; i++)
        {
            bounds.setLow(i, knots.col(i).minCoeff() - margin);
            bounds.setHigh(i, knots.col(i).maxCoeff() + margin);
        }
        space_->setBounds(bounds);

        // Define start and goal states
        ob::ScopedState<> start(space_);
        start[0] = knots(0, 0);
        start[1] = knots(0, 1);
        start[2] = knots(0, 2);

        ob::ScopedState<> goal(space_);
        goal[0] = knots(knots.rows() - 1, 0);
        goal[1] = knots(knots.rows() - 1, 1);
        goal[2] = knots(knots.rows() - 1, 2);

        // SimpleSetup
        og::SimpleSetup ss(space_);
        ss.setStateValidityChecker(std::bind(&SwingTrajOptRRT::isStateValid, this, std::placeholders::_1));
        ss.setStartAndGoalStates(start, goal);
        ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation()));

        // Planner setup
        // auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation())); // FIXME: error
        // auto planner(std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation()));
        planner->setRange(0.1); // max step size
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(max_time);

        if (solved)
        {
            ss.simplifySolution();
            std::cout << "Found solution:" << std::endl;
            ss.getSolutionPath().printAsMatrix(std::cout);
            std::vector<Point3D> poly_path;
            for (std::size_t i = 0; i < ss.getSolutionPath().getStateCount(); ++i)
            {
                const auto *pos = ss.getSolutionPath().getState(i)->as<ob::RealVectorStateSpace::StateType>();
                poly_path.emplace_back(Point3D(pos->values[0], pos->values[1], pos->values[2]));
            }
            traj = std::make_shared<MincoTrajectory>(poly_path, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), config_.trajTime);
            return true;
        }
        else
        {
            std::cout << "No solution found" << std::endl;
            return false;
        }
    }
};

class CfgValidityChecker : public ob::StateValidityChecker
{
private:
    SwingTrajPlannerConfig config_;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    Eigen::Vector3d start_exclude_cylinder_;
    Eigen::Vector3d end_exclude_cylinder_;
    pinocchio::SE3 pose0_;
    pinocchio::SE3 pose1_;
    int index_;

public:
    CfgValidityChecker(const ob::SpaceInformationPtr &si,
                       SwingTrajPlannerConfig &config,
                       std::shared_ptr<ElSpiderAirInterface> robot_interface,
                       std::shared_ptr<GridMapInterface> gridmap_interface,
                       pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                       Eigen::Vector3d start_exclude_cylinder,
                       Eigen::Vector3d end_exclude_cylinder,
                       int index) : ob::StateValidityChecker(si),
                                    config_(config),
                                    robot_interface_(robot_interface),
                                    gridmap_interface_(gridmap_interface),
                                    pose0_(pose0),
                                    pose1_(pose1),
                                    start_exclude_cylinder_(start_exclude_cylinder),
                                    end_exclude_cylinder_(end_exclude_cylinder),
                                    index_(index) {
                                    };
    bool isValid(const ob::State *state) const override
    {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d pos_vec(point_SE3Act(poseLinearInterp(pose0_, pose1_, pos->values[3]).inverse(),
                                             robot_interface_->FK_foot(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]), index_)));

        double sdf = gridmap_interface_->sdfValue(pos_vec, "min");
        if (!inZCylinder(pos_vec, start_exclude_cylinder_, config_.excludeRadius) &&
            !inZCylinder(pos_vec, end_exclude_cylinder_, config_.excludeRadius) &&
            config_.collBallRadius > sdf - config_.collMargin)
        {
            return false;
        }
        if (pos->values[0] < config_.joint1PosMin || pos->values[0] > config_.joint1PosMax ||
            pos->values[1] < config_.joint2PosMin || pos->values[1] > config_.joint2PosMax ||
            pos->values[2] < config_.joint3PosMin || pos->values[2] > config_.joint3PosMax)
            return false;
        return true;
    }

    double clearance(const ob::State *state) const override
    {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d pos_vec(point_SE3Act(poseLinearInterp(pose0_, pose1_, pos->values[3]).inverse(),
                                             robot_interface_->FK_foot(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]), index_)));
        if (inZCylinder(pos_vec, start_exclude_cylinder_, config_.excludeRadius) ||
            inZCylinder(pos_vec, end_exclude_cylinder_, config_.excludeRadius))
        {
            return config_.excludeRadius;
        }
        return gridmap_interface_->sdfValue(pos_vec, "min");
    }
};

class CfgClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    CfgClearanceObjective(const ob::SpaceInformationPtr &si) : ob::StateCostIntegralObjective(si, true)
    {
    }

    ob::Cost stateCost(const ob::State *s) const
    {
        return ob::Cost(0.10 / si_->getStateValidityChecker()->clearance(s));
    }
};

class CfgTimeSequenceObjective : public ob::OptimizationObjective
{
public:
    CfgTimeSequenceObjective(const ob::SpaceInformationPtr &si) : ob::OptimizationObjective(si)
    {
    }

    /** \brief Returns identity cost. */
    ob::Cost stateCost(const ob::State *s) const override
    {
        return ob::Cost(0);
    };

    /** \brief Motion cost for this objective is defined as
        the configuration space distance between \e s1 and \e
        s2, using the method SpaceInformation::distance(). */
    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {
        double max_cost = 100;
        double transition = 0.05;
        double t1 = s1->as<ob::RealVectorStateSpace::StateType>()->values[3];
        double t2 = s2->as<ob::RealVectorStateSpace::StateType>()->values[3];
        double delta = t2 - t1;
        if (delta < 0)
            return ob::Cost(max_cost);
        else if (delta < transition)
        {
            return ob::Cost(max_cost * (transition - delta) / transition);
        }
        else
            return ob::Cost(0);
    };

    /** \brief the motion cost heuristic for this objective is
        simply the configuration space distance between \e s1
        and \e s2, since this is the optimal cost between any
        two states assuming no obstacles. */
    ob::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const override
    {
        return ob::Cost(0);
    };
};

class SwingCfgTrajOptRRT
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    Eigen::Vector3d start_exclude_cylinder_;
    Eigen::Vector3d end_exclude_cylinder_;
    std::shared_ptr<ob::RealVectorStateSpace> space_;
    int index_;
    pinocchio::SE3 pose0_;
    pinocchio::SE3 pose1_;

    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

    // Benchmarking
    Benchmark benchmark_;

public:
    SwingCfgTrajOptRRT(SwingTrajPlannerConfig config,
                       std::shared_ptr<ElSpiderAirInterface> robot_interface,
                       std::shared_ptr<GridMapInterface> gridmap_interface,
                       std::shared_ptr<GCSVisualizer> visualizer = nullptr,
                       bool enable_benchmark = true)
        : config_(config), robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          space_(std::make_shared<ob::RealVectorStateSpace>(4)),
          visualizer_(visualizer),
          benchmark_("SwingCfgTrajOptRRT", enable_benchmark)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;
    };

    // NOTE: not used by RRT-Connect (non optimal planner)
    ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
        ob::OptimizationObjectivePtr clearObj(new CfgClearanceObjective(si));
        ob::OptimizationObjectivePtr timeObj(new CfgTimeSequenceObjective(si));
        // Optimization termination condition
        lengthObj->setCostThreshold(ob::Cost(4.0));
        timeObj->setCostThreshold(ob::Cost(0.2));

        return lengthObj;
    }

    inline bool optimize(std::shared_ptr<TrajectoryBase> &traj,
                         const pinocchio::SE3 &pose0,
                         const pinocchio::SE3 &pose1,
                         int index)
    {
        // Setup Params
        index_ = index;
        Eigen::Vector3d s = traj->evaluate(0, 0, true);
        Eigen::Vector3d g = traj->evaluate(1, 0, true);
        pose0_ = pose0;
        pose1_ = pose1;
        start_exclude_cylinder_ = point_SE3Act(pose0_.inverse(), robot_interface_->FK_foot(s, index_));
        end_exclude_cylinder_ = point_SE3Act(pose1_.inverse(), robot_interface_->FK_foot(g, index_));
        double max_time = config_.maxTime;

        // Set Bounds
        ob::RealVectorBounds bounds(4);
        bounds.setLow(0, config_.joint1PosMin);
        bounds.setHigh(0, config_.joint1PosMax);
        bounds.setLow(1, config_.joint2PosMin);
        bounds.setHigh(1, config_.joint2PosMax);
        bounds.setLow(2, config_.joint3PosMin);
        bounds.setHigh(2, config_.joint3PosMax);
        bounds.setLow(3, 0);
        bounds.setHigh(3, 1); // param t

        space_->setBounds(bounds);

        // Define start and goal states
        ob::ScopedState<> start(space_);
        start[0] = s(0);
        start[1] = s(1);
        start[2] = s(2);
        start[3] = 0;

        ob::ScopedState<> goal(space_);
        goal[0] = g(0);
        goal[1] = g(1);
        goal[2] = g(2);
        goal[3] = 1;

        // SimpleSetup
        og::SimpleSetup ss(space_);
        ob::StateValidityCheckerPtr checker_ptr =
            std::make_shared<CfgValidityChecker>(ss.getSpaceInformation(),
                                                 config_, robot_interface_, gridmap_interface_,
                                                 pose0_, pose1_,
                                                 start_exclude_cylinder_,
                                                 end_exclude_cylinder_, index_);
        ss.setStateValidityChecker(checker_ptr);
        ss.setStartAndGoalStates(start, goal);
        ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation()));

        // Planner setup
        // auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
        auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation())); // FIXME: error
        // auto planner(std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation()));
        planner->setRange(0.4); // max step size
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(max_time);

        if (solved)
        {
            ss.simplifySolution();
            std::cout << "Found solution:" << std::endl;
            ss.getSolutionPath().printAsMatrix(std::cout);
            std::vector<Point3D> points(ss.getSolutionPath().getStateCount());
            std::vector<double> tvec(ss.getSolutionPath().getStateCount());
            Eigen::VectorXd ts(ss.getSolutionPath().getStateCount() - 1);
            for (std::size_t i = 0; i < ss.getSolutionPath().getStateCount(); ++i)
            {
                const auto *pos = ss.getSolutionPath().getState(i)->as<ob::RealVectorStateSpace::StateType>();
                points.at(i) << pos->values[0], pos->values[1], pos->values[2];
                tvec.at(i) = pos->values[3];
            }
            for (std::size_t i = 1; i < ss.getSolutionPath().getStateCount(); ++i)
            {
                ts(i - 1) = tvec.at(i) - tvec.at(i - 1);
                if (ts(i - 1) < 0)
                {
                    std::cout << "Time sequence error" << std::endl;
                    return false;
                }
            }
            traj = std::make_shared<MincoTrajectory>(points, ts);
            return true;
        }
        else
        {
            std::cout << "No solution found" << std::endl;
            return false;
        }
    }
};