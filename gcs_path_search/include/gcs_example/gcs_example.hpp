/**
 * @file gcs_example.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <chrono>
/* external project header files */
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <dynamic_reconfigure/server.h>

/* internal project header files */
#include "gcs_traj_opt/geo_utils/geo_utils.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"

#include "gcs_traj_opt/geo_utils/quickhull.hpp"
#include "gcs_traj_opt/poly_traj/poly_traj_search.hpp"
#include "misc/gcs_visualizer.hpp"
// TODO: Change project name
#include <gcs_path_search/GCSExampleConfig.h>

using namespace geo_utils_2d;

struct GCS_Example_Config
{
    double testRate;
    std::string mapTopic;
    std::string exampleName;

    // eg_guide_surface_demo
    double polyNum;

    inline void loadParameters(const ros::NodeHandle &nh_priv)
    {
        nh_priv.param<double>("testRate", testRate, 4);
        nh_priv.param<std::string>("mapTopic", mapTopic, "grid_map");
        nh_priv.param<std::string>("exampleName", exampleName, "eg_gcs_barrier_demo");
        nh_priv.param<double>("polyNum", polyNum, 3);
        return;
    }
};

class GCS_Example
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher map_pub_;
    grid_map::GridMap map_;
    bool map_received_ = false;

    GCSVisualizer gcs_visualizer_;
    GCS_Example_Config conf_;

    Eigen::Matrix3Xd vPoly = {3, 10}; // Test Default Hull for ElSpider Air

    // Dyn reconf
    dynamic_reconfigure::Server<gcs_path_search::GCSExampleConfig> server;
    dynamic_reconfigure::Server<gcs_path_search::GCSExampleConfig>::CallbackType f;

    Eigen::Vector2d start = {-0.4, -0.3};
    Eigen::MatrixX3d pos_shift = {1, 3};

public:
    GCS_Example(GCS_Example_Config &conf, ros::NodeHandle &nh_);
    ~GCS_Example() = default;

    // Callbacks
    void map_callback(const grid_map_msgs::GridMap::ConstPtr &msg);
    void dyn_reconf_callback(gcs_path_search::GCSExampleConfig &config, uint32_t level);

    // Utils
    Eigen::Vector2d getPos(const GridPt &idx);
    bool gcs_path_search(std::vector<Polyhedra> polys, Point3D start3d, Point3D goal3d, bool use_string_straining);

    // Examples
    void example_run(std::string name);
    void eg_guide_surface();
    void eg_gcs_barrier_demo();
    void eg_gcs_rand_corridor_demo();
    void eg_gcs_rand_map_demo();
};