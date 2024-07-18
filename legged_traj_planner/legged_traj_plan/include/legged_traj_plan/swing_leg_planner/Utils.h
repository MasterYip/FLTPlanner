/**
 * @file Utils.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <iostream>
/* external project header files */
#include <ros/ros.h>
/* internal project header files */

/**
 * @brief Smoothed approximation of max(x, 0)
 *
 * @param[in] x input value
 * @param[in] mu polishment factor
 * @param[out] f output value
 * @param[out] df output derivative
 * @return true value violates constraint
 * @return false value satisfies constraint
 */
static inline bool smoothedL1(const double &x,
                              const double &mu,
                              double &f,
                              double &df)
{
    if (x < 0.0)
    {
        return false;
    }
    else if (x > mu)
    {
        f = x - 0.5 * mu;
        df = 1.0;
        return true;
    }
    else
    {
        const double xdmu = x / mu;
        const double sqrxdmu = xdmu * xdmu;
        const double mumxd2 = mu - 0.5 * x;
        f = mumxd2 * sqrxdmu * xdmu;
        df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return true;
    }
}

struct SwingTrajPlannerConfig
{
    // Traj Init
    double vLift;
    double hLift;
    double trajTime;

    // Planner Select
    int plannerID;
    // 0: LFTPlanner (TaskSpace & CfgSpace)
    // 1: RRTPlanner (TaskSpace)

    //// ID[0] LFTPlannerSettings
    //// GCS TrajSearch & MINCO optimization
    // Enable
    bool enableOptimizer;
    // Minco Init
    bool useCfgSpace;
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

    //// ID[1] RRTPlannerSettings
    double maxTime;
    double collBallRadius;
    double excludeRadius;
    double collMargin;

    // Misc
    bool enableOptVis;
    bool enableBenchmark;
    std::string benchmarkSavePath;
    std::string robotProfilePath;

    void loadParams(ros::NodeHandle &nh)
    {
        bool check_digit = true;
        check_digit *= nh.getParam("plannerID", plannerID);

        check_digit *= nh.getParam("trajInit/vLift", vLift);
        check_digit *= nh.getParam("trajInit/hLift", hLift);
        check_digit *= nh.getParam("trajInit/trajTime", trajTime);

        check_digit *= nh.getParam("misc/enableOptimizer", enableOptimizer);
        check_digit *= nh.getParam("misc/useCfgSpace", useCfgSpace);
        check_digit *= nh.getParam("misc/enableOptVis", enableOptVis);
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
        }
        //// ID[1] RRTPlannerSettings
        else if (plannerID == 1)
        {
            check_digit *= nh.getParam("RRTPlanner/maxTime", maxTime);
            check_digit *= nh.getParam("RRTPlanner/collBallRadius", collBallRadius);
            check_digit *= nh.getParam("RRTPlanner/excludeRadius", excludeRadius);
            check_digit *= nh.getParam("RRTPlanner/collMargin", collMargin);
        }

        if (!check_digit)
        {
            ROS_ERROR("Not all parameters loaded successfully!");
            throw std::exception();
        }
    }
};