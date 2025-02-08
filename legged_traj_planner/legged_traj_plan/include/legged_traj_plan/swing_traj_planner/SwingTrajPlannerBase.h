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
#include "legged_traj_plan/utils/Geometry.h"
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
    int reOptimizeMaxTry;
    bool useCfgSpace;
    bool useCfgCommand;
    bool enableVis;
    bool enableOptVis;
    double optVisRate;
    bool enableBenchmark;
    std::string OptBenchmarkSavePath;
    std::string ReachableBenchmarkSavePath;
    std::string robotProfilePath;

    // Planner Select
    int plannerID;

    //// ID[0] LFTPlannerSettings
    //// GCS TrajSearch & MINCO optimization
    // Seacher Settings
    bool enablePolyPathSearch;
    bool useLeggedBorderCheck;
    bool updateGuideSurfInReachableCheck;
    double collBallCheckRad2; // Knee
    double collBallCheckRad3; // Foot
    // Optimizer Settings
    double lengthPerPiece;
    double allocSpeed;
    double relCostTol;
    double smoothingFactor;
    int integralResolution;
    bool enableSpaceDeform;
    double spaceDeform1;
    double spaceDeform2;
    double spaceDeform3;
    //// Penalty
    double energyWeight;
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
    // Primitive COst
    double primitivePos1;
    double primitivePos2;
    double primitivePos3;
    double primitiveWeight1;
    double primitiveWeight2;
    double primitiveWeight3;

    //// ID[1] RRTPlannerSettings
    double maxTime;
    // double CollBall3Rad;
    double excludeRadius;
    double collMargin;

    //// ID[2] RRTCfgPlannerSettings
    // double maxTime;
    // double CollBall2Rad;
    // double CollBall3Rad;
    // double excludeRadius;
    // double collMargin;
    // Same with above
    // double joint1PosMin;
    // double joint1PosMax;
    // double joint2PosMin;
    // double joint2PosMax;
    // double joint3PosMin;
    // double joint3PosMax;

    //// ID[3] HeightClearPlannerSettings

    //// ID[4] StompPlannerSettings
    // double stompNumTimesteps;
    // double stompStdDev1;
    // double stompStdDev2;
    // double stompStdDev3;
    // int stompNumIters;
    // int stompNumItersAfterValid;
    // int stompNumRollouts;
    // int stompMaxRollouts;
    // double stompExpCostSensitivity;
    // double stompCtrlCostWeight;
    // Penalty
    // double smoothingFactor;
    // double CollBall3Rad;
    // double CollBall3Weight;
    // double FootCollExcludeBallRad;

    //// ID[5] StompCfgPlannerSettings
    double stompNumTimesteps;
    double stompStdDev1;
    double stompStdDev2;
    double stompStdDev3;
    int stompNumIters;
    int stompNumItersAfterValid;
    int stompNumRollouts;
    int stompMaxRollouts;
    double stompExpCostSensitivity;
    double stompCtrlCostWeight;
    //// Penalty
    // double smoothingFactor;
    // double joint1PosMin;
    // double joint1PosMax;
    // double joint2PosMin;
    // double joint2PosMax;
    // double joint3PosMin;
    // double joint3PosMax;
    // double jointPosWeight;
    // // Collision Cost
    // double CollBall1Rad;
    // double CollBall2Rad;
    // double CollBall3Rad;
    // double CollBall1Weight;
    // double CollBall2Weight;
    // double CollBall3Weight;
    // double FootCollExcludeBallRad;
    // double FootCollExcludeBallSmoothRad;

    //// ID[6] FECPlannerSettings
    // double collBallCheckRad2; // Knee
    // double collBallCheckRad3; // Foot
    // double FootCollExcludeBallRad;
    double FECCheckResolution;

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
        check_digit *= nh.getParam("misc/reOptimizeMaxTry", reOptimizeMaxTry);
        check_digit *= nh.getParam("misc/useCfgSpace", useCfgSpace);
        check_digit *= nh.getParam("misc/useCfgCommand", useCfgCommand);
        check_digit *= nh.getParam("misc/enableVis", enableVis);
        check_digit *= nh.getParam("misc/enableOptVis", enableOptVis);
        check_digit *= nh.getParam("misc/optVisRate", optVisRate);
        check_digit *= nh.getParam("misc/enableBenchmark", enableBenchmark);
        check_digit *= nh.getParam("misc/OptBenchmarkSavePath", OptBenchmarkSavePath);
        check_digit *= nh.getParam("misc/ReachableBenchmarkSavePath", ReachableBenchmarkSavePath);
        check_digit *= nh.getParam("misc/robotProfilePath", robotProfilePath);

        //// ID[0] LFTPlannerSettings
        if (plannerID == 0)
        {
            check_digit *= nh.getParam("LFTPlanner/searcher/enablePolyPathSearch", enablePolyPathSearch);
            check_digit *= nh.getParam("LFTPlanner/searcher/useLeggedBorderCheck", useLeggedBorderCheck);
            check_digit *= nh.getParam("LFTPlanner/searcher/updateGuideSurfInReachableCheck", updateGuideSurfInReachableCheck);
            check_digit *= nh.getParam("LFTPlanner/searcher/collBallCheckRad2", collBallCheckRad2);
            check_digit *= nh.getParam("LFTPlanner/searcher/collBallCheckRad3", collBallCheckRad3);
            check_digit *= nh.getParam("LFTPlanner/optimizer/lengthPerPiece", lengthPerPiece);
            check_digit *= nh.getParam("LFTPlanner/optimizer/allocSpeed", allocSpeed);
            check_digit *= nh.getParam("LFTPlanner/optimizer/relCostTol", relCostTol);
            check_digit *= nh.getParam("LFTPlanner/optimizer/smoothingFactor", smoothingFactor);
            check_digit *= nh.getParam("LFTPlanner/optimizer/integralResolution", integralResolution);
            check_digit *= nh.getParam("LFTPlanner/optimizer/enableSpaceDeform", enableSpaceDeform);
            check_digit *= nh.getParam("LFTPlanner/optimizer/spaceDeform1", spaceDeform1);
            check_digit *= nh.getParam("LFTPlanner/optimizer/spaceDeform2", spaceDeform2);
            check_digit *= nh.getParam("LFTPlanner/optimizer/spaceDeform3", spaceDeform3);
            check_digit *= nh.getParam("LFTPlanner/penalty/energyWeight", energyWeight);
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
            check_digit *= nh.getParam("LFTPlanner/penalty/primitivePos1", primitivePos1);
            check_digit *= nh.getParam("LFTPlanner/penalty/primitivePos2", primitivePos2);
            check_digit *= nh.getParam("LFTPlanner/penalty/primitivePos3", primitivePos3);
            check_digit *= nh.getParam("LFTPlanner/penalty/primitiveWeight1", primitiveWeight1);
            check_digit *= nh.getParam("LFTPlanner/penalty/primitiveWeight2", primitiveWeight2);
            check_digit *= nh.getParam("LFTPlanner/penalty/primitiveWeight3", primitiveWeight3);
        }
        //// ID[1] RRTPlannerSettings
        else if (plannerID == 1)
        {
            check_digit *= nh.getParam("RRTPlanner/maxTime", maxTime);
            check_digit *= nh.getParam("RRTPlanner/CollBall3Rad", CollBall3Rad);
            check_digit *= nh.getParam("RRTPlanner/excludeRadius", excludeRadius);
            check_digit *= nh.getParam("RRTPlanner/collMargin", collMargin);
        }
        //// ID[2] RRTCfgPlannerSettings
        else if (plannerID == 2)
        {
            check_digit *= nh.getParam("RRTCfgPlanner/maxTime", maxTime);
            check_digit *= nh.getParam("RRTCfgPlanner/CollBall2Rad", CollBall2Rad);
            check_digit *= nh.getParam("RRTCfgPlanner/CollBall3Rad", CollBall3Rad);
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
        //// ID[4] StompPlannerSettings
        else if (plannerID == 4)
        {
            check_digit *= nh.getParam("StompPlanner/optimizer/stompNumTimesteps", stompNumTimesteps);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompStdDev1", stompStdDev1);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompStdDev2", stompStdDev2);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompStdDev3", stompStdDev3);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompNumIters", stompNumIters);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompNumItersAfterValid", stompNumItersAfterValid);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompNumRollouts", stompNumRollouts);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompMaxRollouts", stompMaxRollouts);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompCtrlCostWeight", stompCtrlCostWeight);
            check_digit *= nh.getParam("StompPlanner/optimizer/stompExpCostSensitivity", stompExpCostSensitivity);

            check_digit *= nh.getParam("StompPlanner/penalty/smoothingFactor", smoothingFactor);

            check_digit *= nh.getParam("StompPlanner/penalty/CollBall3Rad", CollBall3Rad);
            check_digit *= nh.getParam("StompPlanner/penalty/CollBall3Weight", CollBall3Weight);
            check_digit *= nh.getParam("StompPlanner/penalty/FootCollExcludeBallRad", FootCollExcludeBallRad);
            // check_digit *= nh.getParam("StompPlanner/penalty/FootCollExcludeBallSmoothRad", FootCollExcludeBallSmoothRad);
        }
        //// ID[5] StompCfgPlannerSettings
        else if (plannerID == 5)
        {
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompNumTimesteps", stompNumTimesteps);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompStdDev1", stompStdDev1);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompStdDev2", stompStdDev2);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompStdDev3", stompStdDev3);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompNumIters", stompNumIters);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompNumItersAfterValid", stompNumItersAfterValid);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompNumRollouts", stompNumRollouts);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompMaxRollouts", stompMaxRollouts);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompCtrlCostWeight", stompCtrlCostWeight);
            check_digit *= nh.getParam("StompCfgPlanner/optimizer/stompExpCostSensitivity", stompExpCostSensitivity);

            check_digit *= nh.getParam("StompCfgPlanner/penalty/smoothingFactor", smoothingFactor);

            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint1PosMin", joint1PosMin);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint1PosMax", joint1PosMax);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint2PosMin", joint2PosMin);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint2PosMax", joint2PosMax);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint3PosMin", joint3PosMin);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/joint3PosMax", joint3PosMax);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/jointPosWeight", jointPosWeight);

            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall1Rad", CollBall1Rad);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall2Rad", CollBall2Rad);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall3Rad", CollBall3Rad);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall1Weight", CollBall1Weight);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall2Weight", CollBall2Weight);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/CollBall3Weight", CollBall3Weight);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/FootCollExcludeBallRad", FootCollExcludeBallRad);
            check_digit *= nh.getParam("StompCfgPlanner/penalty/FootCollExcludeBallSmoothRad", FootCollExcludeBallSmoothRad);
        }
        //// ID[6] FECPlannerSettings
        else if (plannerID == 6)
        {
            check_digit *= nh.getParam("FECPlanner/collBallCheckRad2", collBallCheckRad2);
            check_digit *= nh.getParam("FECPlanner/collBallCheckRad3", collBallCheckRad3);
            check_digit *= nh.getParam("FECPlanner/FootCollExcludeBallRad", FootCollExcludeBallRad);
            check_digit *= nh.getParam("FECPlanner/FECCheckResolution", FECCheckResolution);
        }
        if (!check_digit)
        {
            ROS_ERROR("Failed to load SwingTrajPlannerConfig.");
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
    std::vector<BenchmarkResult> opt_bm_results_;
    std::vector<BenchmarkResult> reachable_bm_results_;

public:
    SwingTrajPlannerBase(SwingTrajPlannerConfig config,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface,
                         std::shared_ptr<GridMapInterface> gridmap_interface) : robot_interface_(robot_interface),
                                                                                gridmap_interface_(gridmap_interface),
                                                                                config_(config),
                                                                                benchmark_("SwingTrajPlannerBenchmark", config_.enableBenchmark) {};
    ~SwingTrajPlannerBase() = default;

    bool ifEndPointKinValid(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                            const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                            int index)
    {
        auto robot_kin = robot_interface_->getRobotKin();
        Eigen::Vector3d q_i;
        if (!robot_kin.inverseKinConstraint(point_SE3Act(pose0, p0), q_i, index, false) ||
            !robot_kin.inverseKinConstraint(point_SE3Act(pose1, p1), q_i, index, false))
        {
            return false;
        }
        return true;
    }

    bool ifKinValid(const pinocchio::SE3 &pose, const Eigen::Vector3d &p, int index)
    {
        Eigen::Vector3d q_i;
        return robot_interface_->getRobotKin().inverseKinConstraint(point_SE3Act(pose, p), q_i, index, false);
    }

    SwingTrajPlannerConfig &getConfig()
    {
        return config_;
    }

    std::shared_ptr<ElSpiderAirInterface> getRobotInterface()
    {
        return robot_interface_;
    }

    std::shared_ptr<GridMapInterface> getGridMapInterface()
    {
        return gridmap_interface_;
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
        
        // Save Opt Benchmark
        std::ofstream file;
        file.open(config_.OptBenchmarkSavePath);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << config_.OptBenchmarkSavePath << std::endl;
            return;
        }
        file << "normalTime, criticalTime, miscTime, totTime, optRetType, trajLen, trajCtrl" << std::endl;
        for (auto result : opt_bm_results_)
        {
            file << result.normal_tot_time << ", " << result.critic_tot_time << ", " << result.misc_tot_time << ", "
                 << result.tot_time;
            for (auto data : result.custom_data)
            {
                file << ", " << data;
            }
            file << std::endl;
        }
        file.close();
        std::cout << "Opt Benchmark results saved to: " << config_.OptBenchmarkSavePath << std::endl;

        // Save Reachable Benchmark
        file.open(config_.ReachableBenchmarkSavePath);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << config_.ReachableBenchmarkSavePath << std::endl;
            return;
        }
        file << "normalTime, criticalTime, miscTime, totTime, total, reachable, index";
        for (int i = 0; i < reachable_bm_results_[0].custom_data.size() - 3; i++)
        {
            file << ", r" << i;
        }
        file << std::endl;
        for (auto result : reachable_bm_results_)
        {
            file << result.normal_tot_time << ", " << result.critic_tot_time << ", " << result.misc_tot_time << ", "
                 << result.tot_time;
            for (auto data : result.custom_data)
            {
                file << ", " << data;
            }
            file << std::endl;
        }
        file.close();
        std::cout << "Reachable Benchmark results saved to: " << config_.ReachableBenchmarkSavePath << std::endl;
    }

    // Trajectory Init Interface
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

    // Trajectory Optimization Interface
    bool optTraj(std::shared_ptr<TrajectoryBase> &traj,
                 const pinocchio::SE3 &pose0,
                 const pinocchio::SE3 &pose1,
                 int index)
    {
        benchmark_.resetTimer();
        bool ret = optTrajHook(traj, pose0, pose1, index);
        benchmark_.record("optTraj");
        benchmark_.addCustomData(ret); // Return Type
        benchmark_.addCustomData(traj->getTrajLength());
        benchmark_.addCustomData(traj->getTrajControl());
        benchmark_.end();
        opt_bm_results_.emplace_back(benchmark_.getResult());
        return ret;
    }

    virtual bool optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                             const pinocchio::SE3 &pose0,
                             const pinocchio::SE3 &pose1,
                             int index) = 0;

    // Reachability Check Interface
    bool reachableCheck(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                        Eigen::Vector3d p0, uint index,
                        std::vector<Eigen::Vector3d> &footholds,
                        std::vector<bool> &reachable)
    {
        reachable.resize(footholds.size(), false);
        benchmark_.reset();
        bool ret = reachableCheckHook(pose0, pose1, p0, index, footholds, reachable);
        benchmark_.record("reachableCheck");
        benchmark_.addCustomData(footholds.size());                                     // Total Footholds
        benchmark_.addCustomData(std::count(reachable.begin(), reachable.end(), true)); // Reachable Footholds
        benchmark_.addCustomData(index);                                                // Index
        // FIXME: Reachable array
        for (size_t i = 0; i < reachable.size(); i++)
        {
            benchmark_.addCustomData(reachable[i]);
        }
        benchmark_.end();
        reachable_bm_results_.emplace_back(benchmark_.getResult());

        if (config_.enableVis && visualizer_)
        {
            // Draw reachable footholds in green, unreachable in red
            visualizer_->setIdGroup(1);
            std::vector<Point3D> reachable_footholds;
            std::vector<Point3D> unreachable_footholds;

            for (size_t i = 0; i < footholds.size(); i++)
            {
                if (reachable[i])
                    reachable_footholds.emplace_back(footholds[i]);
                // else
                //     unreachable_footholds.emplace_back(footholds[i]);
            }
            visualizer_->visCube(reachable_footholds, Eigen::Vector4d(1, 0, 0, 0), ros_visualizer::VisStyle(0.6, 0.8, 1.0, 1.0, 0.02));
            // visualizer_->visSphere(reachable_footholds, ros_visualizer::VisStyle(0.5, 0.5, 1.0, 1.0, 0.02));
            // visualizer_->visSphere(unreachable_footholds, ros_visualizer::VisStyle(1.0, 0.7, 0.4, 1.0, 0.02));
        }
        return ret;
    };

    virtual bool reachableCheckHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                    Eigen::Vector3d p0, uint index,
                                    std::vector<Eigen::Vector3d> &footholds,
                                    std::vector<bool> &reachable)
    {
        return false;
    };
};