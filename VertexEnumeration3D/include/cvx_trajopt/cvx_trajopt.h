#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
/* internal project header files */
#include "cvx_trajopt/cvx_trajopt_config.hpp"
#include "geo_utils/geo_utils.hpp"
#include "geo_utils/quickhull.hpp"
#include "misc/visualizer.hpp"

class CVX_TrajOpt
{
public:
    CVX_TrajOpt(CVX_TrajOpt_Config &conf, ros::NodeHandle &nh_);
    ~CVX_TrajOpt();
    void map_callback(const grid_map_msgs::GridMap::ConstPtr &msg);

    
    // Test
    void test_map();
    void draw_vpoly_2DinHullPointset();
    

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;

    Visualizer visualizer_;
    CVX_TrajOpt_Config conf_;
    grid_map::GridMap map_;
    bool map_received_ = false;

    Eigen::Matrix3Xd vPoly = {3, 10}; // Test Default Hull
};