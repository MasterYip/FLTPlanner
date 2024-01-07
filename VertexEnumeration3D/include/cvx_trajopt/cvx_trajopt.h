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
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <dynamic_reconfigure/server.h>

/* internal project header files */
#include "cvx_trajopt/cvx_trajopt_config.hpp"
#include "geo_utils/geo_utils.hpp"
#include "geo_utils/quickhull.hpp"
#include "misc/visualizer.hpp"
// TODO: Change project name
#include <polyve/CvxTrajOptConfig.h>

class TimerMixin
{
protected:
    timespec ts{};
    std::chrono::time_point<std::chrono::system_clock> time_point_;

public:
    TimerMixin(){};
    virtual ~TimerMixin() = default;
    void nanoSleep(uint64_t ns)
    {
        ts.tv_sec = ns / 1000000000;
        ts.tv_nsec = ns % 1000000000;
        nanosleep(&ts, NULL);
    }
    void milliSleep(uint64_t ms)
    {
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (ms % 1000) * 1000000;
        nanosleep(&ts, NULL);
    }
    void timerStart(void)
    {
        time_point_ = std::chrono::system_clock::now();
    }

    /**
     * @brief Stop timer and return elapsed time in nanoseconds
     *
     * @return uint64_t elapsed time in nanoseconds
     */
    uint64_t timerStop(void)
    {
        auto time_point_now = std::chrono::system_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::nanoseconds>(time_point_now - time_point_);
        return duration.count();
    }
};
typedef TimerMixin Timer;

class CVX_TrajOpt
{
public:
    CVX_TrajOpt(CVX_TrajOpt_Config &conf, ros::NodeHandle &nh_);
    ~CVX_TrajOpt();
    void map_callback(const grid_map_msgs::GridMap::ConstPtr &msg);
    std::vector<grid_map::Index> getCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                                            const Eigen::Vector2d &start,
                                                            const Eigen::Vector2d &goal,
                                                            const std::string connectivity);
    bool findConcavePoint(const std::vector<grid_map::Index> &Border,
                          const Eigen::Vector2d &start,
                          const Eigen::Vector2d &goal,
                          std::vector<grid_map::Index> ptsSideA,
                          std::vector<grid_map::Index> ptsSideB);

    // Vis
    void drawSphereIdx(const grid_map::Index &idx, const double radius, bool del_all);
    void drawCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                     const Eigen::Vector2d &start, const Eigen::Vector2d &goal);
    // Test
    void test_map();
    void draw_vpoly_2DinHullPointset();
    void drawCorriderIntersectBorderTest();

    void dyn_reconf_callback(polyve::CvxTrajOptConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    Timer timer_;

    Visualizer visualizer_;
    CVX_TrajOpt_Config conf_;
    grid_map::GridMap map_;
    bool map_received_ = false;

    Eigen::Matrix3Xd vPoly = {3, 10}; // Test Default Hull

    // Dyn reconf
    dynamic_reconfigure::Server<polyve::CvxTrajOptConfig> server;
    dynamic_reconfigure::Server<polyve::CvxTrajOptConfig>::CallbackType f;

    // drawCorriderIntersectBorderTest
    Eigen::Vector2d start = {-0.4, -0.3};
    Eigen::MatrixX3d pos_shift = {1, 3};
};