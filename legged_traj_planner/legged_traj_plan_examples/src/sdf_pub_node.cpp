/*
 * sdf_demo_node.cpp
 *
 *  Created on: May 3, 2022
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#include <string>

#include <ros/ros.h>

#include "legged_traj_plan_examples/SdfPub.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_map_sdf_demo");
  ros::NodeHandle nodeHandle("~");

  std::string elevationLayer;
  nodeHandle.param("elevation_layer", elevationLayer, std::string("elevation"));

  std::string mapTopic;
  nodeHandle.param("grid_map_topic", mapTopic, std::string("grid_map"));

  std::string pointcloudTopic;
  nodeHandle.param("pointcloud_topic", pointcloudTopic, std::string("sdf_pointcloud"));

  grid_map_demos::SdfDemo sdfDemo(nodeHandle, mapTopic, elevationLayer, pointcloudTopic);

  ros::spin();
  return 0;
}