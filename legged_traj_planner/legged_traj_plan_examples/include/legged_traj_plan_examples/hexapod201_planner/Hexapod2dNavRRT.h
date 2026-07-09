#pragma once

#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include <Eigen/Dense>
#include <memory>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Hexapod2dNavRRT {
public:
  Hexapod2dNavRRT() = default;
  ~Hexapod2dNavRRT() = default;

  // Plan a 2D path from start to goal using OMPL RRTConnect
  // with traversability-layer-based state validity checking.
  // Falls back to straight-line path on OMPL failure.
  // start, goal: Eigen::Vector2d (x, y) in world frame
  // gridmap: pointer to GridMapInterface (provides traversability map)
  // output_path: vector of Eigen::Vector2d waypoints
  // Returns true always (fallback ensures a path is always produced)
  bool planPath(const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                std::shared_ptr<GridMapInterface> gridmap,
                std::vector<Eigen::Vector2d> &output_path,
                double step_size = 0.2, double timeout = 3.0) const {
    // --- Fallback: simple straight-line path ---
    auto fallback = [&]() {
      output_path.clear();
      double dist = (goal - start).norm();
      int num_steps = std::max(1, static_cast<int>(dist / step_size));
      for (int i = 0; i <= num_steps; ++i) {
        double ratio = static_cast<double>(i) / num_steps;
        output_path.push_back(start + (goal - start) * ratio);
      }
    };

    if (!gridmap) {
      ROS_WARN("Hexapod2dNavRRT: null gridmap, using straight-line fallback.");
      fallback();
      return true;
    }

    try {
      auto space = std::make_shared<ob::RealVectorStateSpace>(2);
      ob::RealVectorBounds bounds(2);

      // Set bounds from gridmap with a small margin
      auto range = gridmap->getRange();
      grid_map::Position position = gridmap->getMap().getPosition();
      double margin = 0.5;
      bounds.setLow(0,  position.x() - range.x() / 2.0 - margin);
      bounds.setLow(1,  position.y() - range.y() / 2.0 - margin);
      bounds.setHigh(0, position.x() + range.x() / 2.0 + margin);
      bounds.setHigh(1, position.y() + range.y() / 2.0 + margin);
      space->setBounds(bounds);

      // State validity: traversable cells have trav > 0.0
      std::string trav_layer = gridmap->getTravLayerName();
      og::SimpleSetup ss(space);
      ss.setStateValidityChecker(
          [gridmap, trav_layer](const ob::State *state) {
            const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
            double x = s->values[0];
            double y = s->values[1];
            double trav =
                gridmap->value(grid_map::Position(x, y), trav_layer);
            // Valid only if traversable (value > 0, not NaN)
            return (!std::isnan(trav) && trav > 0.0);
          });

      ob::ScopedState<> start_state(space);
      start_state[0] = start.x();
      start_state[1] = start.y();
      ob::ScopedState<> goal_state(space);
      goal_state[0] = goal.x();
      goal_state[1] = goal.y();
      ss.setStartAndGoalStates(start_state, goal_state);

      ss.setOptimizationObjective(
          std::make_shared<ob::PathLengthOptimizationObjective>(
              ss.getSpaceInformation()));

      auto planner =
          std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
      planner->setRange(step_size);
      ss.setPlanner(planner);

      if (!ss.solve(timeout)) {
        ROS_WARN("Hexapod2dNavRRT: RRTConnect timeout, using fallback.");
        fallback();
        return true;
      }

      ss.simplifySolution();
      const auto &path_states = ss.getSolutionPath().getStates();
      output_path.clear();
      for (const auto *state : path_states) {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        output_path.emplace_back(s->values[0], s->values[1]);
      }

      ROS_INFO_STREAM("Hexapod2dNavRRT: RRTConnect path ("
                      << output_path.size() << " waypoints, obstacle-aware).");
      return true;

    } catch (const std::exception &e) {
      ROS_WARN("Hexapod2dNavRRT: OMPL exception '%s', using fallback.",
               e.what());
      fallback();
      return true;
    }
  }
};