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
/* external project header files */
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

/* internal project header files */

using Derivative3 = Eigen::Vector3d;

class GridMapInterface
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    grid_map::GridMap map_;
    std::unique_ptr<grid_map::SignedDistanceField> sdf[2];
    std::pair<Eigen::Vector3d, Eigen::Vector3d> sdf_range[2];
    std::string ground_layer = "elevation";
    std::string ceiling_layer = "ceiling";

public:
    GridMapInterface(const std::string &topic_name = "grid_map");

    void callback(const grid_map_msgs::GridMap &msg);
    void update(bool block = true, double sdf_margin = 0.2);
    void updateSDF(const std::string &layer_name, uint index = 0, double margin = 0.2);
    double value(const grid_map::Position &position, const std::string &layer_name = "");
    double sdfValue(const grid_map::Position3 &position, size_t index = 0, const std::string &mode = "min");
    Derivative3 sdfDerivative(const grid_map::Position3 &position, size_t index = 0);
    grid_map::Length getRange() const;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getSdfRange(size_t index = 0) const;
};
