/*
 * ImageToGridmapDemo.hpp
 *
 *  Created on: May 4, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace grid_map_demos
{

  /*!
   * Loads an image and saves it as layer 'elevation' of a grid map.
   * The grid map is published and can be viewed in Rviz.
   */
  class ImageToGridmapDemo
  {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    ImageToGridmapDemo(ros::NodeHandle &nodeHandle,
                                           std::string elevation_layer,
                                           std::string ceiling_layer,
                                           std::string grid_map_topic);

    /*!
     * Destructor.
     */
    virtual ~ImageToGridmapDemo();

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    void imageCallback(const sensor_msgs::Image &msg);

    void imageCeilingCallback(const sensor_msgs::Image &msg);

  private:
    //! ROS nodehandle.
    ros::NodeHandle &nodeHandle_;

    //! Grid map publisher.
    ros::Publisher gridMapPublisher_;

    //! Grid map data.
    grid_map::GridMap map_;

    //! Image subscriber
    ros::Subscriber imageSubscriber_;
    bool withCeiling_;
    ros::Subscriber imageCeilingSubscriber_;
    sensor_msgs::Image ceilingImgBuffer_;

    //! Name of the grid map topic.
    std::string imageTopic_;
    std::string imageCeilingTopic_;

    // Layer Name
    std::string elevation_layer_name_;
    std::string ceiling_layer_name_;

    //! Length of the grid map in x direction.
    double mapLengthX_;

    //! Resolution of the grid map.
    double resolution_;

    //! Range of the height values.
    double minHeight_;
    double maxHeight_;

    //! Frame id of the grid map.
    std::string mapFrameId_;

    bool mapInitialized_;
  };

} /* namespace */
