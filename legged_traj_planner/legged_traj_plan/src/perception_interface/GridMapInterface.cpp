/**
 * @file GridMapInterface.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/perception_interface/GridMapInterface.h"

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

GridMapInterface::GridMapInterface(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   std::string ground_layer_name,
                                   std::string ceiling_layer_name) : nh_(nh),
                                                                     ground_layer(ground_layer_name),
                                                                     ceiling_layer(ceiling_layer_name)
{
    sub_ = nh_.subscribe(topic_name, 1, &GridMapInterface::callback, this);
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_trav_test", 1, true);
    pointcloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/full_sdf", 1);
    freespacePublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/free_space", 1);
    occupiedPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/occupied_space", 1);
    map_.setFrameId("map");
    ground_layer_trav = ground_layer + "_trav";

    update();
}

GridMapInterface::GridMapInterface(ros::NodeHandle &nh,
                                   GridMapInterfaceConfig &config) : nh_(nh),
                                                                     config_(config)

{
    ground_layer = config_.groundLayerName;
    ceiling_layer = config_.ceilingLayerName;
    sub_ = nh_.subscribe(config_.topicName, 1, &GridMapInterface::callback, this);
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_trav_test", 1, true);
    pointcloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/full_sdf", 1);
    freespacePublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/free_space", 1);
    occupiedPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/occupied_space", 1);
    map_.setFrameId("map");
    ground_layer_trav = ground_layer + "_trav";

    update();
}

void GridMapInterface::callback(const grid_map_msgs::GridMap &msg)
{
    if (!map_update_lock_)
    {
        grid_map::GridMapRosConverter::fromMessage(msg, map_);
        update();
    }
}

// FIXME: update might be called only once (SDF will not be updated all the time)
void GridMapInterface::update(bool block, double sdf_margin)
{
    while (map_.getLayers().empty() && block && ros::ok())
    {
        ROS_WARN("GridMap_Interface - Waiting for GridMap message...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    if (!map_recv_flag_)
    {
        map_recv_flag_ = true;
        if (map_.exists(ground_layer))
            ROS_INFO("GridMap_Interface - Ground Layer Initializing...");
        if (map_.exists(ceiling_layer))
            ROS_INFO("GridMap_Interface - Ceiling Layer Initializing...");
    }
    updateSDF(ground_layer, 0, sdf_margin);
    updateTravMap();
    if (map_.exists(ceiling_layer))
    {
        updateSDF(ceiling_layer, 1, sdf_margin);
    }

    //// Debug
    // TravMap Visualization
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    pub_.publish(message);

    // SDF
    sensor_msgs::PointCloud2 pointCloud2Msg;
    grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg);
    pointcloudPublisher_.publish(pointCloud2Msg);
    grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg, 1, [](float sdfValue)
                                                { return sdfValue > 0.0; });
    freespacePublisher_.publish(pointCloud2Msg);
    grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg, 1, [](float sdfValue)
                                                { return sdfValue <= 0.0; });
    occupiedPublisher_.publish(pointCloud2Msg);
}

void GridMapInterface::updateTravMap(void)
{
    try
    {
        map_.add(ground_layer_trav, map_.get(ground_layer));
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            bool valid = true;
            double normal_tan_ = std::sqrt(std::pow(map_.at(ground_norm_x_layer, *iterator), 2) + std::pow(map_.at(ground_norm_y_layer, *iterator), 2)) / map_.at(ground_norm_z_layer, *iterator);
            valid &= normal_tan_ < config_.normalTangentCrtic;
            valid &= config_.enableHeightFilter ? map_.at(ground_layer, *iterator) < config_.maxHeight : true;
            if (!valid)
                map_.at(ground_layer_trav, *iterator) = std::nan("");
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Failed to update trav map!");
    }
}

void GridMapInterface::updateSDF(const std::string &layer_name, uint index, double margin)
{
    try
    {
        // Whole map (SDF seems not accurate)
        // auto &elevationData = map_.get(layer_name);
        // const double minValue{elevationData.minCoeffOfFinites() - margin};
        // const double maxValue{elevationData.maxCoeffOfFinites() + margin};
        // grid_map::SignedDistanceField sdf_temp(map_, layer_name, minValue, maxValue);
        // sdf_range_[index] = std::make_pair(Eigen::Vector3d(0, 0, minValue),
        //                                    Eigen::Vector3d(map_.getLength().x(), map_.getLength().y(), maxValue));
        // sdf_[index] = std::make_unique<grid_map::SignedDistanceField>(sdf_temp);

        // Sub map
        bool ret = false;
        // TODO: use robot center position?
        grid_map::GridMap submap = map_.getSubmap(map_.getPosition(), map_.getLength() * 0.8, ret);
        auto &elevationData = submap.get(layer_name);
        const double minValue{elevationData.minCoeffOfFinites() - margin};
        const double maxValue{elevationData.maxCoeffOfFinites() + margin};
        grid_map::SignedDistanceField sdf_temp(submap, layer_name, minValue, maxValue);
        sdf_range_[index] = std::make_pair(Eigen::Vector3d(0, 0, minValue),
                                           Eigen::Vector3d(submap.getLength().x(), submap.getLength().y(), maxValue));
        sdf_[index] = std::make_unique<grid_map::SignedDistanceField>(sdf_temp);
    }
    catch (const std::out_of_range &e)
    {
        ROS_WARN_STREAM("Layer " << layer_name << " not found!");
    }
}

double GridMapInterface::value(const grid_map::Position &position, const std::string &layer_name)
{
    // Check position validity
    try
    {
        if (layer_name.empty())
            return map_.atPosition(ground_layer, position);
        else
            return map_.atPosition(layer_name, position);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 0.0; // TODO: return interpolation
    }
}

double GridMapInterface::sdfValue(const grid_map::Position3 &position, const std::string &mode)
{
    if (mode == "min")
    {
        if (!sdf_[0])
        {
            ROS_WARN("SDF is not initialized!");
            return 0.0;
        }
        else if (!sdf_[1])
        {
            return sdf_[0]->value(position);
        }
        else
        {
            return std::min(sdf_[0]->value(position), -sdf_[1]->value(position));
        }
    }
    else if (mode == "ground")
    {
        if (!sdf_[0])
        {
            ROS_WARN("SDF is not initialized!");
            return 0.0;
        }
        return sdf_[0]->value(position);
    }
    else
    {
        throw std::invalid_argument("mode should be 'min' or 'ground'");
    }
}

Derivative3 GridMapInterface::sdfDerivative(const grid_map::Position3 &position, size_t index)
{
    if (!sdf_[index])
    {
        ROS_WARN("SDF is not initialized!");
        return Eigen::Vector3d::Zero();
    }
    return sdf_[index]->derivative(position).transpose();
}

grid_map::Length GridMapInterface::getRange() const
{
    return map_.getLength();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> GridMapInterface::getSdfRange(size_t index) const
{
    return sdf_range_[index];
}
