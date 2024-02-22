/*
 * image_to_gridmap_demo_node.cpp
 *
 *  Created on: May 04, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "fast_legged_planner/ImageToGridmap.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap");
  ros::NodeHandle nh("~");
  std::string elevation_layer = nh.param<std::string>("elevation_layer", "elevation");
  std::string ceiling_layer = nh.param<std::string>("ceiling_layer", "ceiling");
  std::string grdi_map_topic = nh.param<std::string>("grid_map_topic", "/grid_map");
  grid_map_demos::ImageToGridmapDemo imageToGridmapDemo(nh, elevation_layer, ceiling_layer, grdi_map_topic);

  ros::spin();
  return 0;
}
