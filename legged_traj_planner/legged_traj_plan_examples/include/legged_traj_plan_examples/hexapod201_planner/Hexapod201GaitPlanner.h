/**
 * @file Hexapod201GaitPlanner.h
 * @author GitHub Copilot
 * @brief Gait planner implementations for Hexapod201 robot
 * @version 0.1
 * @date 2024-08-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include <ros/ros.h>

// Hexapod gait types
enum class HexapodGaitType
{
    GAIT_2, // 2-gait: moves 3 feet at once (tripod)
    GAIT_3, // 3-gait: moves 2 feet at once
    GAIT_6  // 6-gait: moves 1 foot at once
};

// Configuration for basic Raibert foothold planner
struct Hexapod201RaibertFootholdPlannerConfig
{
    double stepTime;
    double stanceTime;
    double velocityGainX;
    double velocityGainY;
    double velocityGainZ;

    void loadParams(ros::NodeHandle &nh, std::string ns = "Hexapod201RaibertFootholdPlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/stepTime", stepTime);
        check_digit &= nh.getParam(ns + "/stanceTime", stanceTime);
        check_digit &= nh.getParam(ns + "/velocityGainX", velocityGainX);
        check_digit &= nh.getParam(ns + "/velocityGainY", velocityGainY);
        check_digit &= nh.getParam(ns + "/velocityGainZ", velocityGainZ);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load Hexapod201RaibertFootholdPlannerConfig.");
        }
    }
};

// Configuration for terrain-aware Raibert planner
struct Hexapod201TerrainAwareRaibertPlannerConfig
{
    double stepTime;
    double stanceTime;
    double velocityGainX;
    double velocityGainY;
    double velocityGainZ;
    double searchRadius;
    double gridResolution;
    int maxSearchIterations;
    double minFootholdHeight;

    void loadParams(ros::NodeHandle &nh, std::string ns = "Hexapod201TerrainAwareRaibertPlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/stepTime", stepTime);
        check_digit &= nh.getParam(ns + "/stanceTime", stanceTime);
        check_digit &= nh.getParam(ns + "/velocityGainX", velocityGainX);
        check_digit &= nh.getParam(ns + "/velocityGainY", velocityGainY);
        check_digit &= nh.getParam(ns + "/velocityGainZ", velocityGainZ);
        check_digit &= nh.getParam(ns + "/searchRadius", searchRadius);
        check_digit &= nh.getParam(ns + "/gridResolution", gridResolution);
        check_digit &= nh.getParam(ns + "/maxSearchIterations", maxSearchIterations);
        check_digit &= nh.getParam(ns + "/minFootholdHeight", minFootholdHeight);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load Hexapod201TerrainAwareRaibertPlannerConfig.");
        }
    }
};

// Main gait planner configuration
struct Hexapod201GaitPlannerConfig
{
    std::string gaitType;
    double stepDuration;
    double stanceDuration;

    // Sub-configurations
    Hexapod201RaibertFootholdPlannerConfig raibertConfig;
    Hexapod201TerrainAwareRaibertPlannerConfig terrainAwareConfig;

    void loadParams(ros::NodeHandle &nh, std::string ns = "GaitPlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/gaitType", gaitType);
        check_digit &= nh.getParam(ns + "/stepDuration", stepDuration);
        check_digit &= nh.getParam(ns + "/stanceDuration", stanceDuration);

        // Load sub-configurations
        raibertConfig.loadParams(nh, ns + "/Hexapod201RaibertFootholdPlanner");
        terrainAwareConfig.loadParams(nh, ns + "/Hexapod201TerrainAwareRaibertPlanner");

        if (!check_digit)
        {
            ROS_ERROR("Failed to load Hexapod201GaitPlannerConfig.");
        }
    }

    HexapodGaitType getGaitTypeEnum() const
    {
        if (gaitType == "GAIT_2")
            return HexapodGaitType::GAIT_2;
        else if (gaitType == "GAIT_3")
            return HexapodGaitType::GAIT_3;
        else if (gaitType == "GAIT_6")
            return HexapodGaitType::GAIT_6;
        else
        {
            ROS_WARN_STREAM("Unknown gait type: " << gaitType << ", defaulting to GAIT_2");
            return HexapodGaitType::GAIT_2;
        }
    }
};

// Tripod gait phases for 2-gait
enum class TripodPhase
{
    PHASE_135, // Legs 1,3,5 (RR, LF, RL) swing, Legs 0,2,4 (RF, FL, LR) stance
    PHASE_246  // Legs 0,2,4 (RF, FL, LR) swing, Legs 1,3,5 (RR, LF, RL) stance
};

// 3-gait phases (wave gait)
enum class WaveGaitPhase
{
    PHASE_01, // Legs 0,1 swing
    PHASE_23, // Legs 2,3 swing
    PHASE_45  // Legs 4,5 swing
};

// 6-gait phases (single leg swing)
enum class SingleLegPhase
{
    LEG_0,
    LEG_1,
    LEG_2,
    LEG_3,
    LEG_4,
    LEG_5
};

// Basic Raibert heuristic for hexapod foothold placement
struct Hexapod201RaibertFootholdPlanner
{
    Hexapod201RaibertFootholdPlannerConfig config_;
    Eigen::Vector3d velocity_gain;

    Hexapod201RaibertFootholdPlanner(const Hexapod201RaibertFootholdPlannerConfig &config)
        : config_(config), velocity_gain(config.velocityGainX, config.velocityGainY, config.velocityGainZ)
    {
    }

    Eigen::Vector3d computeFoothold(const pinocchio::SE3 &body_pose,
                                    const Eigen::Vector3d &body_velocity,
                                    const Eigen::Vector3d &nominal_foothold,
                                    int leg_index)
    {
        // Raibert heuristic: foothold = nominal + velocity_gain * body_velocity *
        // (step_time/2 + stance_time/2)
        double foothold_time = config_.stepTime / 2.0 + config_.stanceTime / 2.0;
        Eigen::Vector3d velocity_offset =
            velocity_gain.cwiseProduct(body_velocity) * foothold_time;

        // Transform nominal foothold to world frame
        Eigen::Vector3d world_nominal =
            point_SE3Act(body_pose.inverse(), nominal_foothold);

        // Add velocity-based offset for forward motion
        return world_nominal + velocity_offset;
    }
};

// Enhanced Raibert foothold planner with terrain awareness
struct Hexapod201TerrainAwareRaibertPlanner
{
    Hexapod201TerrainAwareRaibertPlannerConfig config_;
    Eigen::Vector3d velocity_gain;

    Hexapod201TerrainAwareRaibertPlanner(const Hexapod201TerrainAwareRaibertPlannerConfig &config)
        : config_(config), velocity_gain(config.velocityGainX, config.velocityGainY, config.velocityGainZ)
    {
    }

    Eigen::Vector3d computeOptimalFoothold(const pinocchio::SE3 &body_pose,
                                           const Eigen::Vector3d &body_velocity,
                                           const Eigen::Vector3d &nominal_foothold,
                                           std::shared_ptr<GridMapInterface> gridmap_interface,
                                           int leg_index)
    {
        // Step 1: Compute Raibert heuristic target
        double foothold_time = config_.stepTime / 2.0 + config_.stanceTime / 2.0;
        Eigen::Vector3d velocity_offset = velocity_gain.cwiseProduct(body_velocity) * foothold_time;

        // Transform nominal foothold to world frame
        Eigen::Vector3d world_nominal = point_SE3Act(body_pose.inverse(), nominal_foothold);
        Eigen::Vector3d raibert_target = world_nominal + velocity_offset;

        // Step 2: Search for closest valid foothold area using foothold layer
        grid_map::GridMap &map = gridmap_interface->getMap();
        std::string foothold_layer = gridmap_interface->getFootholdLayerName();

        // Check if foothold layer exists
        if (!map.exists(foothold_layer))
        {
            ROS_WARN_STREAM("Foothold layer '" << foothold_layer << "' does not exist, falling back to ground layer");
            // Fallback to using terrain height from ground layer
            grid_map::Position target_pos(raibert_target[0], raibert_target[1]);
            double terrain_height = gridmap_interface->value(target_pos);
            return Eigen::Vector3d(raibert_target[0], raibert_target[1], terrain_height);
        }

        // Check if target position is already a valid foothold
        grid_map::Position target_pos(raibert_target[0], raibert_target[1]);
        if (map.isInside(target_pos) && !std::isnan(map.atPosition(foothold_layer, target_pos)))
        {
            // Target is a valid foothold, use it directly with foothold layer height
            double foothold_height = map.atPosition(foothold_layer, target_pos);
            return Eigen::Vector3d(raibert_target[0], raibert_target[1], foothold_height);
        }

        // Step 3: Use circle iterator to find closest valid foothold
        double best_distance = std::numeric_limits<double>::max();
        Eigen::Vector3d best_foothold = world_nominal; // Fallback to nominal
        bool found_valid_foothold = false;
        int iterations = 0;

        // Search in expanding circles
        for (double radius = config_.gridResolution; radius <= config_.searchRadius; radius += config_.gridResolution)
        {
            for (grid_map::CircleIterator iterator(map, target_pos, radius);
                 !iterator.isPastEnd() && iterations < config_.maxSearchIterations; ++iterator, ++iterations)
            {
                grid_map::Position current_pos;
                map.getPosition(*iterator, current_pos);

                // Check if this position is a valid foothold
                // FIXME: actually is not score but height, this should be fixed later
                double foothold_height = map.at(foothold_layer, *iterator);
                if (!std::isnan(foothold_height) && foothold_height >= config_.minFootholdHeight)
                {
                    double distance = (current_pos - target_pos).norm();
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        best_foothold = Eigen::Vector3d(current_pos[0], current_pos[1], foothold_height);
                        found_valid_foothold = true;
                    }
                }
            }

            // If we found a valid foothold in this radius, use it
            if (found_valid_foothold)
                break;
        }

        // Step 4: Return result
        if (found_valid_foothold)
        {
            return best_foothold;
        }
        else
        {
            // throw exception
            throw std::runtime_error("No valid foothold found within search radius.");
            ROS_WARN_STREAM("Foothold not found, falling back to default.");
            // No valid foothold found, return nominal with terrain height from ground layer
            grid_map::Position nominal_pos(world_nominal[0], world_nominal[1]);
            double terrain_height = gridmap_interface->value(nominal_pos);
            return Eigen::Vector3d(world_nominal[0], world_nominal[1], terrain_height);
            
        }
    }
};

// Main gait planner class
class Hexapod201GaitPlanner
{
private:
    Hexapod201GaitPlannerConfig config_;
    HexapodGaitType gait_type_;

    // Phase states for different gait types
    TripodPhase tripod_phase_;
    WaveGaitPhase wave_phase_;
    SingleLegPhase single_leg_phase_;

    // Foothold planners
    Hexapod201RaibertFootholdPlanner raibert_planner_;
    Hexapod201TerrainAwareRaibertPlanner terrain_aware_planner_;

    // Gait timing parameters
    double step_duration_;
    double stance_duration_;

public:
    Hexapod201GaitPlanner(const Hexapod201GaitPlannerConfig &config)
        : config_(config),
          gait_type_(config.getGaitTypeEnum()),
          tripod_phase_(TripodPhase::PHASE_135),
          wave_phase_(WaveGaitPhase::PHASE_01),
          single_leg_phase_(SingleLegPhase::LEG_0),
          raibert_planner_(config.raibertConfig),
          terrain_aware_planner_(config.terrainAwareConfig),
          step_duration_(config.stepDuration),
          stance_duration_(config.stanceDuration)
    {
    }

    // Set gait type
    void setGaitType(HexapodGaitType gait_type) { gait_type_ = gait_type; }
    HexapodGaitType getGaitType() const { return gait_type_; }

    // Get contact pattern based on current gait type and phase
    std::array<bool, 6> getCurrentContactPattern()
    {
        switch (gait_type_)
        {
        case HexapodGaitType::GAIT_2:
            return getTripodContactPattern(tripod_phase_);
        case HexapodGaitType::GAIT_3:
            return getWaveGaitContactPattern(wave_phase_);
        case HexapodGaitType::GAIT_6:
            return getSingleLegContactPattern(single_leg_phase_);
        default:
            return getTripodContactPattern(tripod_phase_);
        }
    }

    // Advance to next phase
    void advancePhase()
    {
        switch (gait_type_)
        {
        case HexapodGaitType::GAIT_2:
            switchTripodPhase();
            break;
        case HexapodGaitType::GAIT_3:
            switchWavePhase();
            break;
        case HexapodGaitType::GAIT_6:
            switchSingleLegPhase();
            break;
        }
    }

    // Compute foothold for swing legs using basic Raibert
    Eigen::Vector3d computeFoothold(const pinocchio::SE3 &body_pose,
                                    const Eigen::Vector3d &body_velocity,
                                    const Eigen::Vector3d &nominal_foothold,
                                    int leg_index)
    {
        return raibert_planner_.computeFoothold(body_pose, body_velocity, nominal_foothold, leg_index);
    }

    // Compute foothold for swing legs using terrain-aware planner
    Eigen::Vector3d computeTerrainAwareFoothold(const pinocchio::SE3 &body_pose,
                                                const Eigen::Vector3d &body_velocity,
                                                const Eigen::Vector3d &nominal_foothold,
                                                std::shared_ptr<GridMapInterface> gridmap_interface,
                                                int leg_index)
    {
        return terrain_aware_planner_.computeOptimalFoothold(body_pose, body_velocity,
                                                             nominal_foothold, gridmap_interface, leg_index);
    }

private:
    // Tripod gait (2-gait) patterns
    std::array<bool, 6> getTripodContactPattern(TripodPhase phase)
    {
        std::array<bool, 6> pattern;
        if (phase == TripodPhase::PHASE_135)
        {
            // Legs 1,3,5 (RR, LF, RL) swing, Legs 0,2,4 (RF, FL, LR) stance
            pattern[0] = true;  // RF stance
            pattern[1] = false; // RR swing
            pattern[2] = true;  // FL stance
            pattern[3] = false; // LF swing
            pattern[4] = true;  // LR stance
            pattern[5] = false; // RL swing
        }
        else // PHASE_246
        {
            // Legs 0,2,4 (RF, FL, LR) swing, Legs 1,3,5 (RR, LF, RL) stance
            pattern[0] = false; // RF swing
            pattern[1] = true;  // RR stance
            pattern[2] = false; // FL swing
            pattern[3] = true;  // LF stance
            pattern[4] = false; // LR swing
            pattern[5] = true;  // RL stance
        }
        return pattern;
    }

    void switchTripodPhase()
    {
        tripod_phase_ = (tripod_phase_ == TripodPhase::PHASE_135)
                            ? TripodPhase::PHASE_246
                            : TripodPhase::PHASE_135;
    }

    // Wave gait (3-gait) patterns - 2 legs swing at a time
    std::array<bool, 6> getWaveGaitContactPattern(WaveGaitPhase phase)
    {
        std::array<bool, 6> pattern = {true, true, true, true, true, true}; // Default all stance

        switch (phase)
        {
        case WaveGaitPhase::PHASE_01:
            pattern[0] = false; // RF swing
            pattern[5] = false; // RR swing
            break;
        case WaveGaitPhase::PHASE_23:
            pattern[1] = false; // FL swing
            pattern[4] = false; // LF swing
            break;
        case WaveGaitPhase::PHASE_45:
            pattern[2] = false; // LR swing
            pattern[3] = false; // RL swing
            break;
        }
        return pattern;
    }

    void switchWavePhase()
    {
        switch (wave_phase_)
        {
        case WaveGaitPhase::PHASE_01:
            wave_phase_ = WaveGaitPhase::PHASE_23;
            break;
        case WaveGaitPhase::PHASE_23:
            wave_phase_ = WaveGaitPhase::PHASE_45;
            break;
        case WaveGaitPhase::PHASE_45:
            wave_phase_ = WaveGaitPhase::PHASE_01;
            break;
        }
    }

    // Single leg gait (6-gait) patterns - 1 leg swing at a time
    std::array<bool, 6> getSingleLegContactPattern(SingleLegPhase phase)
    {
        std::array<bool, 6> pattern = {true, true, true, true, true, true}; // Default all stance

        switch (phase)
        {
        case SingleLegPhase::LEG_0:
            pattern[0] = false; // RF swing
            break;
        case SingleLegPhase::LEG_1:
            pattern[1] = false; // RR swing
            break;
        case SingleLegPhase::LEG_2:
            pattern[2] = false; // FL swing
            break;
        case SingleLegPhase::LEG_3:
            pattern[3] = false; // LF swing
            break;
        case SingleLegPhase::LEG_4:
            pattern[4] = false; // LR swing
            break;
        case SingleLegPhase::LEG_5:
            pattern[5] = false; // RL swing
            break;
        }
        return pattern;
    }

    void switchSingleLegPhase()
    {
        switch (single_leg_phase_)
        {
        case SingleLegPhase::LEG_0:
            single_leg_phase_ = SingleLegPhase::LEG_1;
            break;
        case SingleLegPhase::LEG_1:
            single_leg_phase_ = SingleLegPhase::LEG_2;
            break;
        case SingleLegPhase::LEG_2:
            single_leg_phase_ = SingleLegPhase::LEG_3;
            break;
        case SingleLegPhase::LEG_3:
            single_leg_phase_ = SingleLegPhase::LEG_4;
            break;
        case SingleLegPhase::LEG_4:
            single_leg_phase_ = SingleLegPhase::LEG_5;
            break;
        case SingleLegPhase::LEG_5:
            single_leg_phase_ = SingleLegPhase::LEG_0;
            break;
        }
    }
};