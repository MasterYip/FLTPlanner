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
#include <grid_map_core/GridMapMath.hpp>

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
    ground_layer_foothold = ground_layer + "_foothold";

    update();
}

GridMapInterface::GridMapInterface(ros::NodeHandle &nh,
                                   GridMapInterfaceConfig &config) : nh_(nh),
                                                                     config_(config)
{
    ground_layer = config_.groundLayerName;
    ceiling_layer = config_.ceilingLayerName;
    sub_ = nh_.subscribe(config_.topicName, 1, &GridMapInterface::callback, this);
    if (config_.topicNameCeiling != config_.topicName)
        sub_ceiling_ = nh_.subscribe(config_.topicNameCeiling, 1, &GridMapInterface::callback_ceiling, this);

    pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_trav_test", 1, true);
    pointcloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/full_sdf", 1);
    freespacePublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/free_space", 1);
    occupiedPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/occupied_space", 1);
    map_.setFrameId("map");
    map_ceiling_.setFrameId("map");
    ground_layer_trav = ground_layer + "_trav";
    ground_layer_foothold = ground_layer + "_foothold";

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

void GridMapInterface::callback_ceiling(const grid_map_msgs::GridMap &msg)
{
    if (!map_update_lock_ && map_.exists(ground_layer))
    {
        grid_map::GridMapRosConverter::fromMessage(msg, map_ceiling_);
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
    // Sensor ceiling map
    if (map_ceiling_.exists(ceiling_layer) && !map_ceiling_.get(ceiling_layer).hasNaN())
        map_.add(ceiling_layer, map_ceiling_.get(ceiling_layer));
    if (!map_recv_flag_)
    {
        map_recv_flag_ = true;
        if (map_.exists(ground_layer))
            ROS_INFO("GridMap_Interface - Ground Layer Initializing...");
        if (map_.exists(ceiling_layer))
            ROS_INFO("GridMap_Interface - Ceiling Layer Initializing...");
    }
    updateTravMap();
    updateFootholdMap();
    if (config_.sdfEnable) // Check if SDF updates are enabled
    {
        updateSDF(ground_layer, 0, sdf_margin);
        if (map_.exists(ceiling_layer))
        {
            updateSDF(ceiling_layer, 1, sdf_margin);
        }
    }

    //// Debug
    // TravMap Visualization
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    pub_.publish(message);

    // SDF
    // sensor_msgs::PointCloud2 pointCloud2Msg;
    // grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg);
    // pointcloudPublisher_.publish(pointCloud2Msg);
    // grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg, 1, [](float sdfValue)
    //                                             { return sdfValue > 0.0; });
    // freespacePublisher_.publish(pointCloud2Msg);
    // grid_map::GridMapRosConverter::toPointCloud(*sdf_[0], pointCloud2Msg, 1, [](float sdfValue)
    //                                             { return sdfValue <= 0.0; });
    // occupiedPublisher_.publish(pointCloud2Msg);
}

void GridMapInterface::updateTravMap(void)
{
    try
    {
        map_.add(ground_layer_trav, map_.get(ground_layer));

        // First pass: apply traversability criteria
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            bool valid = true;
            double normal_tan_ = std::sqrt(std::pow(map_.at(ground_norm_x_layer, *iterator), 2) + std::pow(map_.at(ground_norm_y_layer, *iterator), 2)) / map_.at(ground_norm_z_layer, *iterator);
            valid &= normal_tan_ < config_.normalTangentCrtic;
            valid &= config_.enableHeightFilter ? map_.at(ground_layer, *iterator) < config_.maxHeight && map_.at(ground_layer, *iterator) > config_.minHeight : true;
            if (!valid)
                map_.at(ground_layer_trav, *iterator) = std::nan("");
        }

        // Second pass: apply erosion if travErodeRad > 0
        if (config_.travErodeRad > 0.0)
        {
            grid_map::Matrix travLayerCopy = map_.get(ground_layer_trav);

            for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
            {
                if (!std::isnan(map_.at(ground_layer_trav, *iterator)))
                {
                    // Check if any cell within erosion radius is invalid
                    grid_map::Position center;
                    map_.getPosition(*iterator, center);

                    bool shouldErode = false;
                    for (grid_map::CircleIterator circleIterator(map_, center, config_.travErodeRad);
                         !circleIterator.isPastEnd(); ++circleIterator)
                    {
                        if (std::isnan(travLayerCopy((*circleIterator)(0), (*circleIterator)(1))))
                        {
                            shouldErode = true;
                            break;
                        }
                    }

                    if (shouldErode)
                    {
                        map_.at(ground_layer_trav, *iterator) = std::nan("");
                    }
                }
            }
        }

        // Third pass: ensure center box area is always traversable
        if (config_.centerBoxAlwaysTrav && config_.centerBoxWidth > 0.0 && config_.centerBoxLen > 0.0)
        {
            grid_map::Position mapCenter = map_.getPosition();
            grid_map::Length boxSize(config_.centerBoxLen, config_.centerBoxWidth);

            // Get start index and size for submap iterator using grid_map namespace function
            grid_map::Index submapTopLeftIndex;
            grid_map::Size submapBufferSize;
            grid_map::Position submapPosition;
            grid_map::Length submapLength;
            grid_map::Index requestedIndexInSubmap;

            bool isValidSubmap = grid_map::getSubmapInformation(
                submapTopLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap,
                mapCenter, boxSize, map_.getLength(), map_.getPosition(), map_.getResolution(),
                map_.getSize(), map_.getStartIndex());

            if (isValidSubmap)
            {
                for (grid_map::SubmapIterator iterator(map_, submapTopLeftIndex, submapBufferSize);
                     !iterator.isPastEnd(); ++iterator)
                {
                    if (!std::isnan(map_.at(ground_layer, *iterator)))
                    {
                        map_.at(ground_layer_trav, *iterator) = map_.at(ground_layer, *iterator);
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Failed to update trav map!");
    }
}

void GridMapInterface::updateFootholdMap(void)
{
    try
    {
        map_.add(ground_layer_foothold, map_.get(ground_layer));

        // First pass: apply foothold traversability criteria
        for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
        {
            bool valid = true;
            double normal_tan_ = std::sqrt(std::pow(map_.at(ground_norm_x_layer, *iterator), 2) + std::pow(map_.at(ground_norm_y_layer, *iterator), 2)) / map_.at(ground_norm_z_layer, *iterator);
            valid &= normal_tan_ < config_.footholdNormalTangentCrtic;
            valid &= config_.footholdEnableHeightFilter ? map_.at(ground_layer, *iterator) < config_.footholdMaxHeight && map_.at(ground_layer, *iterator) > config_.footholdMinHeight : true;
            if (!valid)
                map_.at(ground_layer_foothold, *iterator) = std::nan("");
        }

        // Second pass: apply erosion if footholdErodeRad > 0
        if (config_.footholdErodeRad > 0.0)
        {
            grid_map::Matrix footholdLayerCopy = map_.get(ground_layer_foothold);

            for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
            {
                if (!std::isnan(map_.at(ground_layer_foothold, *iterator)))
                {
                    // Check if any cell within erosion radius is invalid
                    grid_map::Position center;
                    map_.getPosition(*iterator, center);

                    bool shouldErode = false;
                    for (grid_map::CircleIterator circleIterator(map_, center, config_.footholdErodeRad);
                         !circleIterator.isPastEnd(); ++circleIterator)
                    {
                        if (std::isnan(footholdLayerCopy((*circleIterator)(0), (*circleIterator)(1))))
                        {
                            shouldErode = true;
                            break;
                        }
                    }

                    if (shouldErode)
                    {
                        map_.at(ground_layer_foothold, *iterator) = std::nan("");
                    }
                }
            }
        }

        // Third pass: ensure center box area is always traversable for foothold
        if (config_.footholdCenterBoxAlwaysTrav && config_.footholdCenterBoxWidth > 0.0 && config_.footholdCenterBoxLen > 0.0)
        {
            grid_map::Position mapCenter = map_.getPosition();
            grid_map::Length boxSize(config_.footholdCenterBoxLen, config_.footholdCenterBoxWidth);

            // Get start index and size for submap iterator using grid_map namespace function
            grid_map::Index submapTopLeftIndex;
            grid_map::Size submapBufferSize;
            grid_map::Position submapPosition;
            grid_map::Length submapLength;
            grid_map::Index requestedIndexInSubmap;

            bool isValidSubmap = grid_map::getSubmapInformation(
                submapTopLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap,
                mapCenter, boxSize, map_.getLength(), map_.getPosition(), map_.getResolution(),
                map_.getSize(), map_.getStartIndex());

            if (isValidSubmap)
            {
                for (grid_map::SubmapIterator iterator(map_, submapTopLeftIndex, submapBufferSize);
                     !iterator.isPastEnd(); ++iterator)
                {
                    if (!std::isnan(map_.at(ground_layer, *iterator)))
                    {
                        map_.at(ground_layer_foothold, *iterator) = map_.at(ground_layer, *iterator);
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Failed to update foothold map!");
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
            return sdf_[0]->value(position);
        else
            return std::min(sdf_[0]->value(position), -sdf_[1]->value(position));
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
        throw std::invalid_argument("mode should be 'min' or 'ground'");
}

/**
 * @brief Get the sdf derivative of the map that is closest to the position
 *
 *
 * @param position
 * @return Derivative3
 */
Derivative3 GridMapInterface::minSdfDerivative(const grid_map::Position3 &position)
{
    if (!sdf_[0])
    {
        ROS_WARN("SDF is not initialized!");
        return Eigen::Vector3d::Zero();
    }
    else
    {
        if (!sdf_[1])
            return sdf_[0]->derivative(position).transpose();
        else
        {
            if (sdf_[0]->value(position) < -sdf_[1]->value(position))
                return sdf_[0]->derivative(position).transpose();
            else
                return -sdf_[1]->derivative(position).transpose();
        }
    }
}

Derivative3 GridMapInterface::sdfDerivative(const grid_map::Position3 &position, size_t index)
{
    if (!sdf_[index])
    {
        ROS_WARN("SDF is not initialized!");
        return Eigen::Vector3d::Zero();
    }
    if (index == 0)
        return sdf_[0]->derivative(position).transpose();
    else
        return -sdf_[1]->derivative(position).transpose(); // ceiling
}

grid_map::Length GridMapInterface::getRange() const
{
    return map_.getLength();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> GridMapInterface::getSdfRange(size_t index) const
{
    return sdf_range_[index];
}
