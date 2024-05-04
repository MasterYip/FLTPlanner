/**
 * @file GridMapInterface.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <iostream>
#include <memory>
#include <atomic>
/* external project header files */
#include <ros/ros.h>
#include <filters/filter_chain.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
// #include <grid_map_filters/MeanInRadiusFilter.hpp>

/* internal project header files */

using Derivative3 = Eigen::Vector3d;

class GridMapInterface
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    grid_map::GridMap map_;
    std::unique_ptr<grid_map::SignedDistanceField> sdf_[2];
    std::pair<Eigen::Vector3d, Eigen::Vector3d> sdf_range_[2];

    std::string filterChainParametersName_; 
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    std::string ground_layer;
    std::string ground_norm_x_layer = {"normal_x"};
    std::string ground_norm_y_layer = {"normal_y"};
    std::string ground_norm_z_layer = {"normal_z"};
    std::string ground_layer_trav;
    std::string ceiling_layer;
    std::string torso_ref_layer = {"torso_ref"};
    std::atomic<bool> map_update_lock_{false};

    ros::Publisher pub_;

public:
    GridMapInterface(ros::NodeHandle &nh,
                     std::string topic_name = "grid_map",
                     std::string ground_layer_name = "elevation_inpainted",
                     std::string ceiling_layer_name = "ceiling");

    void callback(const grid_map_msgs::GridMap &msg);
    void update(bool block = true, double sdf_margin = 0.2);
    void updateTorsoRef(void);
    void updateTravMap(void);
    void updateSDF(const std::string &layer_name, uint index = 0, double margin = 0.2);
    double value(const grid_map::Position &position, const std::string &layer_name = "");
    double sdfValue(const grid_map::Position3 &position, const std::string &mode = "min");
    Derivative3 sdfDerivative(const grid_map::Position3 &position, size_t index = 0);
    grid_map::Length getRange() const;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getSdfRange(size_t index = 0) const;
    grid_map::GridMap &getMap() { return map_; };
    std::string getGroundLayerName() { return ground_layer; };
    std::string getCeilingLayerName() { return ceiling_layer; };
    // Map Lock
    void lockMapUpdate() { map_update_lock_ = true; }
    void unlockMapUpdate() { map_update_lock_ = false; }
    bool isMapUpdateLocked() { return map_update_lock_; };
};
