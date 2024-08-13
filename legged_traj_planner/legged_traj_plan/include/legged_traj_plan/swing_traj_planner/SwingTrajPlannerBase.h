/**
 * @file SwingTrajPlannerBase.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
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
#include "legged_traj_search/utils/benchmark.hpp"

struct SwingTrajPlannerConfig
{
    // Traj Init
    double vLift;
    double hLift;
    double trajTime;
    bool enableLiftRandomize{false}; // Enabled when replanning
    double vLiftNormalRandomize;

    // Misc
    bool enableOptimizer;
    bool reOptimize;
    bool useCfgSpace;
    bool useCfgCommand;
    bool enableVis;
    bool enableOptVis;
    double optVisRate;
    bool enableBenchmark;
    std::string benchmarkSavePath;
    std::string robotProfilePath;

    // Planner Select
    int plannerID;
    // 0: LFTPlanner (TaskSpace & CfgSpace)
    // 1: RRTPlanner (TaskSpace)

    //// ID[0] LFTPlannerSettings
    //// GCS TrajSearch & MINCO optimization
    // Enable
    // Minco Init
    double lengthPerPiece;
    double allocSpeed;
    // Optimizer Settings
    double relCostTol;
    double smoothingFactor;
    int integralResolution;
    //// Penalty
    // Time Cost
    double timeWeight;
    // Joint Limit Cost
    double joint1PosMin;
    double joint1PosMax;
    double joint2PosMin;
    double joint2PosMax;
    double joint3PosMin;
    double joint3PosMax;
    double jointMaxVel;
    double jointMaxAcc;
    double jointPosWeight;
    double jointVelWeight;
    double jointAccWeight;
    // Collision Cost
    double CollBall1Rad;
    double CollBall2Rad;
    double CollBall3Rad;
    double CollBall1Weight;
    double CollBall2Weight;
    double CollBall3Weight;
    double FootCollExcludeBallRad;
    double FootCollExcludeBallSmoothRad;

    //// ID[1] RRTPlannerSettings
    double maxTime;
    double collBallRadius;
    double excludeRadius;
    double collMargin;

    //// ID[2] RRTCfgPlannerSettings
    // double maxTime;
    // double collBallRadius;
    // double excludeRadius;
    // double collMargin;
    // Same with above
    // double joint1PosMin;
    // double joint1PosMax;
    // double joint2PosMin;
    // double joint2PosMax;
    // double joint3PosMin;
    // double joint3PosMax;

    void loadParams(ros::NodeHandle &nh)
    {
        bool check_digit = true;
        check_digit *= nh.getParam("plannerID", plannerID);

        check_digit *= nh.getParam("trajInit/vLift", vLift);
        check_digit *= nh.getParam("trajInit/hLift", hLift);
        check_digit *= nh.getParam("trajInit/trajTime", trajTime);
        check_digit *= nh.getParam("trajInit/vLiftNormalRandomize", vLiftNormalRandomize);

        check_digit *= nh.getParam("misc/enableOptimizer", enableOptimizer);
        check_digit *= nh.getParam("misc/reOptimize", reOptimize);
        check_digit *= nh.getParam("misc/useCfgSpace", useCfgSpace);
        check_digit *= nh.getParam("misc/useCfgCommand", useCfgCommand);
        check_digit *= nh.getParam("misc/enableVis", enableVis);
        check_digit *= nh.getParam("misc/enableOptVis", enableOptVis);
        check_digit *= nh.getParam("misc/optVisRate", optVisRate);
        check_digit *= nh.getParam("misc/enableBenchmark", enableBenchmark);
        check_digit *= nh.getParam("misc/benchmarkSavePath", benchmarkSavePath);
        check_digit *= nh.getParam("misc/robotProfilePath", robotProfilePath);

        //// ID[0] LFTPlannerSettings
        if (plannerID == 0)
        {
            check_digit *= nh.getParam("LFTPlanner/optimizer/lengthPerPiece", lengthPerPiece);
            check_digit *= nh.getParam("LFTPlanner/optimizer/allocSpeed", allocSpeed);
            check_digit *= nh.getParam("LFTPlanner/optimizer/relCostTol", relCostTol);
            check_digit *= nh.getParam("LFTPlanner/optimizer/smoothingFactor", smoothingFactor);
            check_digit *= nh.getParam("LFTPlanner/optimizer/integralResolution", integralResolution);
            check_digit *= nh.getParam("LFTPlanner/penalty/timeWeight", timeWeight);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint1PosMin", joint1PosMin);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint1PosMax", joint1PosMax);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint2PosMin", joint2PosMin);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint2PosMax", joint2PosMax);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint3PosMin", joint3PosMin);
            check_digit *= nh.getParam("LFTPlanner/penalty/joint3PosMax", joint3PosMax);
            check_digit *= nh.getParam("LFTPlanner/penalty/jointMaxVel", jointMaxVel);
            check_digit *= nh.getParam("LFTPlanner/penalty/jointMaxAcc", jointMaxAcc);
            check_digit *= nh.getParam("LFTPlanner/penalty/jointPosWeight", jointPosWeight);
            check_digit *= nh.getParam("LFTPlanner/penalty/jointVelWeight", jointVelWeight);
            check_digit *= nh.getParam("LFTPlanner/penalty/jointAccWeight", jointAccWeight);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall1Rad", CollBall1Rad);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall2Rad", CollBall2Rad);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall3Rad", CollBall3Rad);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall1Weight", CollBall1Weight);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall2Weight", CollBall2Weight);
            check_digit *= nh.getParam("LFTPlanner/penalty/CollBall3Weight", CollBall3Weight);
            check_digit *= nh.getParam("LFTPlanner/penalty/FootCollExcludeBallRad", FootCollExcludeBallRad);
            check_digit *= nh.getParam("LFTPlanner/penalty/FootCollExcludeBallSmoothRad", FootCollExcludeBallSmoothRad);
        }
        //// ID[1] RRTPlannerSettings
        else if (plannerID == 1)
        {
            check_digit *= nh.getParam("RRTPlanner/maxTime", maxTime);
            check_digit *= nh.getParam("RRTPlanner/collBallRadius", collBallRadius);
            check_digit *= nh.getParam("RRTPlanner/excludeRadius", excludeRadius);
            check_digit *= nh.getParam("RRTPlanner/collMargin", collMargin);
        }
        //// ID[2] RRTCfgPlannerSettings
        else if (plannerID == 2)
        {
            check_digit *= nh.getParam("RRTCfgPlanner/maxTime", maxTime);
            check_digit *= nh.getParam("RRTCfgPlanner/collBallRadius", collBallRadius);
            check_digit *= nh.getParam("RRTCfgPlanner/excludeRadius", excludeRadius);
            check_digit *= nh.getParam("RRTCfgPlanner/collMargin", collMargin);
            check_digit *= nh.getParam("RRTCfgPlanner/joint1PosMin", joint1PosMin);
            check_digit *= nh.getParam("RRTCfgPlanner/joint1PosMax", joint1PosMax);
            check_digit *= nh.getParam("RRTCfgPlanner/joint2PosMin", joint2PosMin);
            check_digit *= nh.getParam("RRTCfgPlanner/joint2PosMax", joint2PosMax);
            check_digit *= nh.getParam("RRTCfgPlanner/joint3PosMin", joint3PosMin);
            check_digit *= nh.getParam("RRTCfgPlanner/joint3PosMax", joint3PosMax);
        }
        //// ID[3] HeightClearPlannerSettings
        else if (plannerID == 3)
        {
            
        }
        if (!check_digit)
        {
            ROS_ERROR("Not all parameters loaded successfully!");
            throw std::exception();
        }
    }
};

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
        file << "normalTime, criticalTime, miscTime, totTime, optRetType, totTime" << std::endl;
        for (auto result : benchmark_results_)
        {
            file << result.normal_tot_time << ", " << result.critic_tot_time << ", " << result.misc_tot_time << ", "
                 << result.tot_time;
            for (auto data : result.custom_data)
            {
                file << ", " << data;
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
        benchmark_.addCustomData(ret);
        benchmark_.addCustomData(traj->getTotalDuration());
        benchmark_.end();
        benchmark_results_.emplace_back(benchmark_.getResult());
        return ret;
    }

    virtual bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                             const pinocchio::SE3 &pose0,
                             const pinocchio::SE3 &pose1,
                             int index) = 0;
};