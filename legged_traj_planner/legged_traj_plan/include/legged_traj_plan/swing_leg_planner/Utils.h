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

/* external project header files */

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
    //// Optimizer
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

    // Misc
    bool enableOptVis;
    bool enableBenchmark;
    std::string benchmarkSavePath;

    void loadParams(ros::NodeHandle &nh)
    {
        nh.param("trajInit/vLift", vLift, 0.2);
        nh.param("trajInit/hLift", hLift, 0.1);
        nh.param("trajInit/trajTime", trajTime, 1.0);
        nh.param("optimizer/enableOptimizer", enableOptimizer, true);
        nh.param("optimizer/useCfgSpace", useCfgSpace, true);
        nh.param("optimizer/lengthPerPiece", lengthPerPiece, 0.6);
        nh.param("optimizer/allocSpeed", allocSpeed, 1.0);
        nh.param("optimizer/relCostTol", relCostTol, 1.0e-2);
        nh.param("optimizer/smoothingFactor", smoothingFactor, 1.0e-2);
        nh.param("optimizer/integralResolution", integralResolution, 16);
        nh.param("penalty/timeWeight", timeWeight, 0.00005);
        nh.param("penalty/joint1PosMin", joint1PosMin, -0.785);
        nh.param("penalty/joint1PosMax", joint1PosMax, 0.785);
        nh.param("penalty/joint2PosMin", joint2PosMin, -0.5233);
        nh.param("penalty/joint2PosMax", joint2PosMax, 3.14);
        nh.param("penalty/joint3PosMin", joint3PosMin, -0.6978);
        nh.param("penalty/joint3PosMax", joint3PosMax, 3.925);
        nh.param("penalty/jointMaxVel", jointMaxVel, 5.0);
        nh.param("penalty/jointMaxAcc", jointMaxAcc, 10.0);
        nh.param("penalty/jointPosWeight", jointPosWeight, 0.4);
        nh.param("penalty/jointVelWeight", jointVelWeight, 0.1);
        nh.param("penalty/jointAccWeight", jointAccWeight, 0.1);
        nh.param("penalty/CollBall1Rad", CollBall1Rad, 0.12);
        nh.param("penalty/CollBall2Rad", CollBall2Rad, 0.12);
        nh.param("penalty/CollBall3Rad", CollBall3Rad, 0.03);
        nh.param("penalty/CollBall1Weight", CollBall1Weight, 0.0);
        nh.param("penalty/CollBall2Weight", CollBall2Weight, 0.0);
        nh.param("penalty/CollBall3Weight", CollBall3Weight, 0.05);
        nh.param("penalty/FootCollExcludeBallRad", FootCollExcludeBallRad, 0.05);
        nh.param("misc/enableOptVis", enableOptVis, false);
        nh.param("misc/enableBenchmark", enableBenchmark, false);
        nh.param("misc/benchmarkSavePath", benchmarkSavePath, std::string(""));
    }
};