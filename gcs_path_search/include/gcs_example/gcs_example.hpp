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
#include "misc/visualizer.hpp"
#include "misc/gcs_visualizer.hpp"
// TODO: Change project name
#include <gcs_path_search/GCSExampleConfig.h>

using namespace geo_utils_2d;

struct GCS_Example_Config
{
    std::string mapTopic;
    double testRate;

    inline void loadParameters(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TestRate", testRate);
        return;
    }
};

class GCS_Example
{
public:
    GCS_Example(GCS_Example_Config &conf, ros::NodeHandle &nh_);
    ~GCS_Example();
    void map_callback(const grid_map_msgs::GridMap::ConstPtr &msg);
    void dyn_reconf_callback(gcs_path_search::GCSExampleConfig &config, uint32_t level);

    bool minlengthPath(const std::vector<GridPt> &Border,
                       const Eigen::Vector2d &start,
                       const Eigen::Vector2d &goal,
                       std::vector<GridPt> &path);

    Eigen::Vector2d getPos(const GridPt &idx);
    bool gcs_path_search(std::vector<Polyhedra> polys, Point3D start3d, Point3D goal3d);

    // Test
    void test_map();
    void schematic_drawer();
    void segmentIntersectTest();
    void drawCorriderIntersectBorderTest();
    void testGCSPathSearch();

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;

    GCSVisualizer gcs_visualizer_;
    GCS_Example_Config conf_;
    grid_map::GridMap map_;
    bool map_received_ = false;
    ros::Publisher map_pub_;

    Eigen::Matrix3Xd vPoly = {3, 10}; // Test Default Hull for ElSpider Air

    // Dyn reconf
    dynamic_reconfigure::Server<gcs_path_search::GCSExampleConfig> server;
    dynamic_reconfigure::Server<gcs_path_search::GCSExampleConfig>::CallbackType f;

    Eigen::Vector2d start = {-0.4, -0.3};
    Eigen::MatrixX3d pos_shift = {1, 3};
};