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

GridMapInterface::GridMapInterface(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   std::string ground_layer_name,
                                   std::string ceiling_layer_name) : nh_(nh),
                                                                     ground_layer(ground_layer_name),
                                                                     ceiling_layer(ceiling_layer_name),
                                                                     filter_chain_("grid_map::GridMap")
{
    sub_ = nh_.subscribe(topic_name, 1, &GridMapInterface::callback, this);
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_trav_test", 1, true);
    map_.setFrameId("map");
    ground_layer_trav = ground_layer + "_trav";

    nh_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));

    // Setup filter chain.
    if (!filter_chain_.configure(filterChainParametersName_, nh_))
    {
        ROS_ERROR("Could not configure the filter chain!");
    }

    update();
}

void GridMapInterface::callback(const grid_map_msgs::GridMap &msg)
{
    if (!map_update_lock_)
    {
        grid_map::GridMapRosConverter::fromMessage(msg, map_);
        // if (!sdf_[0])
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
    updateTravMap();
    // FIXME: disabled for performance reasons
    // if (!filter_chain_.update(map_, map_))
    // {
    //     ROS_ERROR("Could not update the grid map filter chain!");
    // }
    // Visualization
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    pub_.publish(message);

    if (map_.exists(ceiling_layer))
    {
        updateSDF(ceiling_layer, 1, sdf_margin);
    }
}

void GridMapInterface::updateTorsoRef(void)
{
    // Add this layer by applying grid_map filter to ground layer & increase z value by nominal torso height
    try
    {
        map_.add(torso_ref_layer, map_.get(ground_layer));
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            map_.at(torso_ref_layer, *iterator) += 0.3;
        }
        // Filter
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Failed to update torso ref layer!");
    }
}

void GridMapInterface::updateTravMap(void)
{
    try
    {
        map_.add(ground_layer_trav, map_.get(ground_layer));
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            double normal_tan_ = std::sqrt(std::pow(map_.at(ground_norm_x_layer, *iterator), 2) + std::pow(map_.at(ground_norm_y_layer, *iterator), 2)) / map_.at(ground_norm_z_layer, *iterator);
            if (normal_tan_ > 0.5)
            {
                map_.at(ground_layer_trav, *iterator) = std::nan("");
            }
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
        auto &elevationData = map_.get(layer_name);
        const double minValue{elevationData.minCoeffOfFinites() - margin};
        const double maxValue{elevationData.maxCoeffOfFinites() + margin};
        grid_map::SignedDistanceField sdf_temp(map_, layer_name, minValue, maxValue);
        sdf_range_[index] = std::make_pair(Eigen::Vector3d(0, 0, minValue),
                                           Eigen::Vector3d(map_.getLength().x(), map_.getLength().y(), maxValue));
        sdf_[index] = std::make_unique<grid_map::SignedDistanceField>(sdf_temp);
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
            // ROS_WARN("Ceiling SDF is not initialized!");
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
