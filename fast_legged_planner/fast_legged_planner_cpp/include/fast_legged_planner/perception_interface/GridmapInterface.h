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
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

/* internal project header files */

class GridMapInterface
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    grid_map::GridMap grid_map;
    std::unique_ptr<grid_map::SignedDistanceField> sdf[2];
    std::string ground_layer = "elevation";
    std::string ceiling_layer = "ceiling";

public:
    GridMapInterface(const std::string &topic_name = "grid_map")
    {
        sub = nh.subscribe(topic_name, 1, &GridMapInterface::callback, this);
        grid_map.setFrameId("map");
        sdf[0] = sdf[1] = nullptr;
    }

    void callback(const grid_map_msgs::GridMap &msg)
    {
        grid_map = grid_map::GridMap(msg);
        if (!sdf[0])
        {
            update();
        }
    }

    void update(bool block = true, double sdf_margin = 0.2)
    {
        while (grid_map.getLayers().empty() && block)
        {
            ROS_WARN("GridMap_Interface - GridMap is not subscribed!");
            ros::Duration(0.5).sleep();
        }
        updateSDF(ground_layer, 0, sdf_margin);
        if (grid_map.exists(ceiling_layer))
        {
            updateSDF(ceiling_layer, 1, sdf_margin);
        }
    }

    void updateSDF(const std::string &layer_name, size_t index, double margin = 0.2)
    {
        try
        {
            grid_map::SignedDistanceField sdf_temp(grid_map, layer_name, margin);
            sdf[index] = std::make_unique<grid_map::SignedDistanceField>(sdf_temp);
            // Update sdf_range[index]
        }
        catch (const std::out_of_range &e)
        {
            ROS_WARN_STREAM("Layer " << layer_name << " not found!");
        }
    }

    double value(const grid_map::Position &position, const std::string &layer_name = "")
    {
        if (layer_name.empty())
        {
            layer_name = ground_layer;
        }
        return grid_map.atPosition(layer_name, position);
    }

    double sdfValue(const grid_map::Position &position, size_t index = 0, const std::string &mode = "min")
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
                return sdf[0]->getValue(position);
            }
            else
            {
                return std::min(sdf[0]->getValue(position), -sdf[1]->getValue(position));
            }
        }
        else if (mode == "ground") // ground only
        {
            if (!sdf[index])
            {
                ROS_WARN("SDF is not initialized!");
                return 0.0;
            }
            return sdf[index]->getValue(position);
        }
        else
        {
            throw std::invalid_argument("mode should be 'min' or 'ground'");
        }
    }

    Eigen::Vector3d sdfDerivative(const grid_map::Position &position, size_t index = 0)
    {
        if (!sdf[index])
        {
            ROS_WARN("SDF is not initialized!");
            return Eigen::Vector3d::Zero();
        }
        return sdf[index]->getGradient(position).transpose();
    }

    grid_map::Length getRange() const
    {
        return grid_map.getLength();
    }

};


