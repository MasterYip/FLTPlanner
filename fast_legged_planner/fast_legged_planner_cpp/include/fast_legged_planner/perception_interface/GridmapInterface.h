/**
 * @file GridmapInterface.h
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
    GridMapInterface(const std::string &topic_name = "grid_map") : nh("~")
    {
        sub = nh.subscribe(topic_name, 1, &GridMapInterface::callback, this);
        map_.setFrameId("map");
        nh.param("elevation_layer", ground_layer, ground_layer);
        nh.param("ceiling_layer", ceiling_layer, ceiling_layer);
        // NOTE: update in blocking mode first
        update();
    }

    void callback(const grid_map_msgs::GridMap &msg)
    {
        grid_map::GridMapRosConverter::fromMessage(msg, map_);
        if (!sdf[0])
            update();
    }

    void update(bool block = true, double sdf_margin = 0.2)
    {
        while (map_.getLayers().empty() && block && ros::ok())
        {
            ROS_WARN("GridMap_Interface - Waiting for GridMap message...");
            ros::spinOnce();
            ros::Duration(2).sleep();
        }
        updateSDF(ground_layer, 0, sdf_margin);
        if (map_.exists(ceiling_layer))
        {
            updateSDF(ceiling_layer, 1, sdf_margin);
        }
    }

    void updateSDF(const std::string &layer_name, uint index = 0, double margin = 0.2)
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

    double value(const grid_map::Position &position, const std::string &layer_name = "")
    {
        if (layer_name.empty())
            return map_.atPosition(ground_layer, position);
        else
            return map_.atPosition(layer_name, position);
    }

    double sdfValue(const grid_map::Position3 &position, size_t index = 0, const std::string &mode = "min")
    {
        if (mode == "min") // min of ground and ceiling
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
        else if (mode == "ground") // ground only
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

    Derivative3 sdfDerivative(const grid_map::Position3 &position, size_t index = 0)
    {
        if (!sdf[index])
        {
            ROS_WARN("SDF is not initialized!");
            return Eigen::Vector3d::Zero();
        }
        return sdf[index]->derivative(position).transpose();
    }

    grid_map::Length getRange() const
    {
        return map_.getLength();
    }
};
