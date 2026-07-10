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

  // Plan a 2D path from start to goal using the traversability layer
  // start, goal: Eigen::Vector2d (x, y)
  // gridmap: pointer to GridMapInterface
  // output_path: vector of Eigen::Vector2d
  // Returns true if path found
  bool planPath(const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                std::shared_ptr<GridMapInterface> gridmap,
                std::vector<Eigen::Vector2d> &output_path,
                double step_size = 0.2, double timeout = 2.0) const {
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    // Set bounds from gridmap (or use large default)
    // print gridmap range
    auto range = gridmap->getRange();
    auto position = gridmap->getMap().getPosition();
    printf("GridMap range: [%f, %f]\n", range.x(), range.y());
    printf("GridMap position: [%f, %f]\n", position.x(), position.y());
    bounds.setLow(0, position.x() - range.x() / 2);
    bounds.setLow(1, position.y() - range.y() / 2);
    bounds.setHigh(0, position.x() + range.x() / 2);
    bounds.setHigh(1, position.y() + range.y() / 2);

    space->setBounds(bounds);
    // Create space information
    og::SimpleSetup ss(space);
    // State validity checker: traversability > threshold
    ss.setStateValidityChecker([gridmap](const ob::State *state) {
      const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
      double x = s->values[0];
      double y = s->values[1];
      // Use traversability layer (e.g. "traversability" or
      // "elevation_inpainted")
      double trav =
          gridmap->value(grid_map::Position(x, y), gridmap->getTravLayerName());
      return !std::isnan(trav);
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
    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    planner->setRange(step_size);
    ss.setPlanner(planner);
    if (!ss.solve(timeout))
      return false;
    ss.simplifySolution();
    const auto &path = ss.getSolutionPath().getStates();
    output_path.clear();
    for (const auto *state : path) {
      const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
      output_path.emplace_back(s->values[0], s->values[1]);
    }
    return true;
  }
//   bool planPath(const Eigen::Vector2d &start, 
//               const Eigen::Vector2d &goal,
//               std::shared_ptr<GridMapInterface> gridmap,  // 留着也不影响，反正不用了
//               std::vector<Eigen::Vector2d> &output_path,
//               double step_size = 0.2) const {
    
//     // 1. 直接清空输出，生成直线（完全忽略地图）
//     output_path.clear();
//     double dist = (goal - start).norm();
//     // 至少生成1段，避免距离为0时路径为空
//     int num_steps = std::max(1, static_cast<int>(dist / step_size));
    
//     for (int i = 0; i <= num_steps; ++i) {
//         double ratio = static_cast<double>(i) / num_steps;
//         output_path.push_back(start + (goal - start) * ratio);
//     }
    
//     return true; 
//}
};