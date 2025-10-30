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

struct GridMapInterfaceConfig
{
    std::string topicName;
    std::string groundLayerName;
    std::string topicNameCeiling;
    std::string ceilingLayerName;

    // SDF
    double sdfMargin{0.3};
    bool sdfEnable{true}; // New configuration to enable/disable SDF updates

    // TravMap
    double normalTangentCrtic{0.5};
    bool enableHeightFilter{false};
    double maxHeight{0.0};
    double minHeight{0.0};

    void loadParam(ros::NodeHandle &nh, std::string ns = "GridMapInterface")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/topicName", topicName);
        check_digit &= nh.getParam(ns + "/groundLayerName", groundLayerName);
        check_digit &= nh.getParam(ns + "/topicNameCeiling", topicNameCeiling);
        check_digit &= nh.getParam(ns + "/ceilingLayerName", ceilingLayerName);
        check_digit &= nh.getParam(ns + "/sdfMargin", sdfMargin);
        check_digit &= nh.getParam(ns + "/normalTangentCrtic", normalTangentCrtic);
        check_digit &= nh.getParam(ns + "/enableHeightFilter", enableHeightFilter);
        check_digit &= nh.getParam(ns + "/maxHeight", maxHeight);
        check_digit &= nh.getParam(ns + "/minHeight", minHeight);
        check_digit &= nh.getParam(ns + "/sdfEnable", sdfEnable); // Load sdfEnable parameter
    }
};

class GridMapInterface
{
private:
    ros::NodeHandle nh_;

    GridMapInterfaceConfig config_;

    ros::Subscriber sub_;
    grid_map::GridMap map_;
    ros::Subscriber sub_ceiling_;
    grid_map::GridMap map_ceiling_;
    std::unique_ptr<grid_map::SignedDistanceField> sdf_[2];
    std::pair<Eigen::Vector3d, Eigen::Vector3d> sdf_range_[2];

    std::string ground_layer;
    std::string ground_norm_x_layer = {"normal_x"};
    std::string ground_norm_y_layer = {"normal_y"};
    std::string ground_norm_z_layer = {"normal_z"};
    std::string ground_layer_trav;
    std::string ceiling_layer;
    std::string torso_ref_layer = {"torso_ref"};
    std::atomic<bool> map_update_lock_{false};
    bool map_recv_flag_ = false;

    //// Debug
    // TravMap publisher
    ros::Publisher pub_;
    // Pointcloud publisher.
    ros::Publisher pointcloudPublisher_;
    // Free space publisher.
    ros::Publisher freespacePublisher_;
    // Occupied space publisher.
    ros::Publisher occupiedPublisher_;

public:
    [[deprecated]] GridMapInterface(ros::NodeHandle &nh,
                                    std::string topic_name = "grid_map",
                                    std::string ground_layer_name = "elevation_inpainted",
                                    std::string ceiling_layer_name = "ceiling");

    GridMapInterface(ros::NodeHandle &nh, GridMapInterfaceConfig &config);

    void callback(const grid_map_msgs::GridMap &msg);
    void callback_ceiling(const grid_map_msgs::GridMap &msg);
    void update(bool block = true, double sdf_margin = 0.3); // FIXME: this should larger than robot height?
    void updateTravMap(void);
    void updateSDF(const std::string &layer_name, uint index = 0, double margin = 0.2);
    double value(const grid_map::Position &position, const std::string &layer_name = "");
    double sdfValue(const grid_map::Position3 &position, const std::string &mode = "min");
    Derivative3 minSdfDerivative(const grid_map::Position3 &position);
    Derivative3 sdfDerivative(const grid_map::Position3 &position, size_t index = 0);
    grid_map::Length getRange() const;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getSdfRange(size_t index = 0) const;
    grid_map::GridMap &getMap() { return map_; };
    std::string getGroundLayerName() { return ground_layer; };
    std::string getTravLayerName() { return ground_layer_trav; };
    std::string getCeilingLayerName() { return ceiling_layer; };
    // Map Lock
    void lockMapUpdate() { map_update_lock_ = true; }
    void unlockMapUpdate() { map_update_lock_ = false; }
    bool isMapUpdateLocked() { return map_update_lock_; };
    bool isCeilingLayerExist() { return map_.exists(ceiling_layer); };
};
