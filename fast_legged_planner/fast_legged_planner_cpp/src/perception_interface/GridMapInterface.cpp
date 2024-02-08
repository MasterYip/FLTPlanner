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

#include "fast_legged_planner/perception_interface/GridMapInterface.h"

GridMapInterface::GridMapInterface(const std::string &topic_name) : nh("~")
{
    sub = nh.subscribe(topic_name, 1, &GridMapInterface::callback, this);
    map_.setFrameId("map");
    nh.param("elevation_layer", ground_layer, ground_layer);
    nh.param("ceiling_layer", ceiling_layer, ceiling_layer);
    update();
}

void GridMapInterface::callback(const grid_map_msgs::GridMap &msg)
{
    if (!map_update_lock_)
    {
        grid_map::GridMapRosConverter::fromMessage(msg, map_);
        if (!sdf[0])
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
    updateSDF(ground_layer, 0, sdf_margin);
    if (map_.exists(ceiling_layer))
    {
        updateSDF(ceiling_layer, 1, sdf_margin);
    }
}

void GridMapInterface::updateSDF(const std::string &layer_name, uint index, double margin)
{
    try
    {
        auto &elevationData = map_.get(layer_name);
        const double minValue{elevationData.minCoeffOfFinites() - margin};
        const double maxValue{elevationData.maxCoeffOfFinites() + margin};
        grid_map::SignedDistanceField sdf_temp(map_, layer_name, minValue, maxValue);
        sdf_range[index] = std::make_pair(Eigen::Vector3d(0, 0, minValue),
                                          Eigen::Vector3d(map_.getLength().x(), map_.getLength().y(), maxValue));
        sdf[index] = std::make_unique<grid_map::SignedDistanceField>(sdf_temp);
    }
    catch (const std::out_of_range &e)
    {
        ROS_WARN_STREAM("Layer " << layer_name << " not found!");
    }
}

double GridMapInterface::value(const grid_map::Position &position, const std::string &layer_name)
{
    if (layer_name.empty())
        return map_.atPosition(ground_layer, position);
    else
        return map_.atPosition(layer_name, position);
}

double GridMapInterface::sdfValue(const grid_map::Position3 &position, size_t index, const std::string &mode)
{
    if (mode == "min")
    {
        if (!sdf[0])
        {
            ROS_WARN("SDF is not initialized!");
            return 0.0;
        }
        else if (!sdf[1])
        {
            ROS_WARN("Ceiling SDF is not initialized!");
            return sdf[0]->value(position);
        }
        else
        {
            return std::min(sdf[0]->value(position), -sdf[1]->value(position));
        }
    }
    else if (mode == "ground")
    {
        if (!sdf[index])
        {
            ROS_WARN("SDF is not initialized!");
            return 0.0;
        }
        return sdf[index]->value(position);
    }
    else
    {
        throw std::invalid_argument("mode should be 'min' or 'ground'");
    }
}

Derivative3 GridMapInterface::sdfDerivative(const grid_map::Position3 &position, size_t index)
{
    if (!sdf[index])
    {
        ROS_WARN("SDF is not initialized!");
        return Eigen::Vector3d::Zero();
    }
    return sdf[index]->derivative(position).transpose();
}

grid_map::Length GridMapInterface::getRange() const
{
    return map_.getLength();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> GridMapInterface::getSdfRange(size_t index) const
{
    return sdf_range[index];
}
