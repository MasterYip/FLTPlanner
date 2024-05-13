/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "legged_traj_plan_examples/ImageToGridmap.hpp"

namespace grid_map_demos
{

  ImageToGridmapDemo::ImageToGridmapDemo(ros::NodeHandle &nodeHandle,
                                         std::string elevation_layer = "elevation",
                                         std::string ceiling_layer = "ceiling",
                                         std::string grid_map_topic = "/grid_map")
      : nodeHandle_(nodeHandle),
        elevation_layer_name_(elevation_layer),
        ceiling_layer_name_(ceiling_layer),
        map_(grid_map::GridMap({elevation_layer})),
        mapInitialized_(false)
  {
    readParameters();
    map_.setBasicLayers({elevation_layer});
    imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmapDemo::imageCallback, this);
    imageCeilingSubscriber_ = nodeHandle_.subscribe(imageCeilingTopic_, 1, &ImageToGridmapDemo::imageCeilingCallback, this);
    gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_topic, 1, true); // "/grid_map" is under root, "grid_map" is under node name
  }

  ImageToGridmapDemo::~ImageToGridmapDemo()
  {
  }

  bool ImageToGridmapDemo::readParameters()
  {
    nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
    nodeHandle_.param("with_ceiling", withCeiling_, false);
    nodeHandle_.param("image_ceiling_topic", imageCeilingTopic_, std::string("/image_ceiling"));
    nodeHandle_.param("resolution", resolution_, 0.03);
    nodeHandle_.param("min_height", minHeight_, 0.0);
    nodeHandle_.param("max_height", maxHeight_, 1.0);
    nodeHandle_.param("min_height_ceiling", minHeightCeiling_, 0.0);
    nodeHandle_.param("max_height_ceiling", maxHeightCeiling_, 1.0);
    nodeHandle_.param("map_frame_id", mapFrameId_, std::string("odom"));
    return true;
  }

  void ImageToGridmapDemo::imageCallback(const sensor_msgs::Image &msg)
  {
    if (!mapInitialized_)
    {
      grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
      ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
               map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
      mapInitialized_ = true;
    }
    if (!ceilingImgBuffer_.data.empty() && withCeiling_)
    {
      grid_map::GridMapRosConverter::addLayerFromImage(ceilingImgBuffer_, ceiling_layer_name_, map_, minHeightCeiling_, maxHeightCeiling_);
      grid_map::GridMapRosConverter::addLayerFromImage(msg, elevation_layer_name_, map_, minHeight_, maxHeight_);
      grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);
      map_.setFrameId(mapFrameId_);
      map_.add("normal_x");
      map_.add("normal_y");
      map_.add("normal_z");

      // Publish as grid map.
      grid_map_msgs::GridMap mapMessage;
      grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
      gridMapPublisher_.publish(mapMessage);
    }
    else if (!withCeiling_)
    {
      grid_map::GridMapRosConverter::addLayerFromImage(msg, elevation_layer_name_, map_, minHeight_, maxHeight_);
      grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);
      map_.setFrameId(mapFrameId_);
      map_.add("normal_x");
      map_.add("normal_y");
      map_.add("normal_z");

      // Publish as grid map.
      grid_map_msgs::GridMap mapMessage;
      grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
      gridMapPublisher_.publish(mapMessage);
    }
  }

  void ImageToGridmapDemo::imageCeilingCallback(const sensor_msgs::Image &msg)
  {
    ceilingImgBuffer_ = msg;
  }

} /* namespace */
