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
#include "cvx_trajopt/cvx_trajopt_config.hpp"

#include "gcs_traj_opt/geo_utils/geo_utils.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"

#include "gcs_traj_opt/geo_utils/quickhull.hpp"
#include "gcs_traj_opt/poly_traj/poly_traj_search.hpp"
#include "misc/visualizer.hpp"
#include "misc/gcs_visualizer.hpp"
// TODO: Change project name
#include <gcs_path_search/CvxTrajOptConfig.h>

using namespace geo_utils_2d;

class CVX_TrajOpt
{
public:
    CVX_TrajOpt(CVX_TrajOpt_Config &conf, ros::NodeHandle &nh_);
    ~CVX_TrajOpt();
    void map_callback(const grid_map_msgs::GridMap::ConstPtr &msg);
    void dyn_reconf_callback(gcs_path_search::CvxTrajOptConfig &config, uint32_t level);

    bool minlengthPath(const std::vector<GridPt> &Border,
                       const Eigen::Vector2d &start,
                       const Eigen::Vector2d &goal,
                       std::vector<GridPt> &path);

    // Vis
    Eigen::Vector2d getPos(const GridPt &idx);

    // Test
    void test_map();
    void schematic_drawer();
    void segmentIntersectTest();
    void drawCorriderIntersectBorderTest();

    // Benchmark
    void benchmarkInit();
    void benchmarkCheck(std::string prefix = "Elapsed", bool algo = false);
    void benchmarkEnd();

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    Timer timer_;

    GCSVisualizer gcs_visualizer_;
    CVX_TrajOpt_Config conf_;
    grid_map::GridMap map_;
    bool map_received_ = false;
    ros::Publisher map_pub_;

    Eigen::Matrix3Xd vPoly = {3, 10}; // Test Default Hull for ElSpider Air

    // Dyn reconf
    dynamic_reconfigure::Server<gcs_path_search::CvxTrajOptConfig> server;
    dynamic_reconfigure::Server<gcs_path_search::CvxTrajOptConfig>::CallbackType f;

    Eigen::Vector2d start = {-0.4, -0.3};
    Eigen::MatrixX3d pos_shift = {1, 3};

    // Benchmark
    double tot_time = 0;
    double algo_time = 0;
    double period_time = 0;
};