/**
 * @file FLTPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

#include "SwingTrajOpt.h"

using namespace geo_utils;

class SwingTrajPlannerBase
{
protected:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    std::shared_ptr<GCSVisualizer> visualizer_;
    Benchmark benchmark_;
    std::vector<BenchmarkResult> benchmark_results_;

public:
    SwingTrajPlannerBase(SwingTrajPlannerConfig config,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface,
                         std::shared_ptr<GridMapInterface> gridmap_interface) : robot_interface_(robot_interface),
                                                                                gridmap_interface_(gridmap_interface),
                                                                                config_(config),
                                                                                benchmark_("SwingTrajPlannerBenchmark", config_.enableBenchmark) {};
    ~SwingTrajPlannerBase() = default;

    SwingTrajPlannerConfig &getConfig()
    {
        return config_;
    }

    std::shared_ptr<ElSpiderAirInterface> getRobotInterface()
    {
        return robot_interface_;
    }

    void visClear()
    {
        if (visualizer_)
            visualizer_->delAll();
    }

    void saveBenchmarkResults()
    {
        if (!config_.enableBenchmark)
            return;
        std::ofstream file;
        file.open(config_.benchmarkSavePath);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << config_.benchmarkSavePath << std::endl;
            return;
        }
        file << "normalTime, criticalTime, miscTime, totTime, minCostFunctional, optRetType" << std::endl;
        for (auto result : benchmark_results_)
        {
            file << result.normal_tot_time << ", " << result.critic_tot_time << ", " << result.misc_tot_time << ", "
                 << result.tot_time << ", ";
            for (auto data : result.custom_data)
            {
                file << data << ", ";
            }
            file << std::endl;
        }
        std::cout << "Benchmark results saved to: " << config_.benchmarkSavePath << std::endl;
    }

    std::shared_ptr<TrajectoryBase> getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                uint index)
    {
        benchmark_.reset();
        std::shared_ptr<TrajectoryBase> traj = getInitTrajHook(pose0, pose1, p0, p1, index);
        benchmark_.record("getInitTraj");
        return traj;
    }

    virtual std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                            Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                            uint index) = 0;

    bool optTraj(std::shared_ptr<TrajectoryBase> &traj,
                 const pinocchio::SE3 &pose0,
                 const pinocchio::SE3 &pose1,
                 int index)
    {
        benchmark_.resetTimer();
        bool ret = optTrajHook(traj, pose0, pose1, index);
        benchmark_.record("optTraj");
        benchmark_.end();
        benchmark_results_.emplace_back(benchmark_.getResult());
        return ret;
    }

    virtual bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                             const pinocchio::SE3 &pose0,
                             const pinocchio::SE3 &pose1,
                             int index) = 0;
};

class FLTPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    SwingTrajOpt swing_traj_opt_;

public:
    FLTPlanner(SwingTrajPlannerConfig config,
               std::shared_ptr<ElSpiderAirInterface> robot_interface,
               std::shared_ptr<GridMapInterface> gridmap_interface);

    /**
     * @brief Search for a poly feasible trajectory
     *
     * @param[out] poly_traj
     * @param pose0
     * @param pose1
     * @param p0
     * @param p1
     * @param index
     * @param verbose
     * @return true
     * @return false
     */
    bool searchPolyTraj(std::vector<Point3D> &poly_traj,
                        const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                        const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                        uint index, bool verbose = true);

    std::shared_ptr<MincoTrajectory> getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                    double v_lift, double h_lift = 0.1);

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index) override;

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index) override;
};

class FLTCfgPlanner : public SwingTrajPlannerBase
{
private:
    ros::NodeHandle nh_;
    SwingTrajOpt swing_traj_opt_;

public:
    FLTCfgPlanner(SwingTrajPlannerConfig config,
                  std::shared_ptr<ElSpiderAirInterface> robot_interface,
                  std::shared_ptr<GridMapInterface> gridmap_interface);

    void visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
                         std::vector<Point3D> cfg_poly_traj,
                         Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
                         int groupId = 1);

    /**
     * @brief Search for a poly feasible trajectory
     *
     * @param[out] poly_traj
     * @param pose0
     * @param pose1
     * @param p0
     * @param p1
     * @param index
     * @param verbose
     * @return true
     * @return false
     */
    bool searchPolyTraj(std::vector<Point3D> &poly_traj,
                        const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                        const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                        uint index, bool verbose = true);

    bool getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                        pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                        Eigen::Vector3d p0, Eigen::Vector3d p1,
                        uint index);

    std::shared_ptr<MincoTrajectory> getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                       const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index);

    std::shared_ptr<TrajectoryBase> getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    uint index) override;

    bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                     const pinocchio::SE3 &pose0,
                     const pinocchio::SE3 &pose1,
                     int index) override;
};