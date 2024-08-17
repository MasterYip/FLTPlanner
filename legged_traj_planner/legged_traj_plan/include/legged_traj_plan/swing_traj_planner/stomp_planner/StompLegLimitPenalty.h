/**
 * @file StompLegLimitPenalty.h
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
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/Utils.h"

class StompLegLimitPenalty
{
private:
    // Config
    Eigen::Matrix<double, 3, 2> posBd_; // [min_pos, max_pos]
    double weight_;                     // weight_pos
    double mu_;                         // Smooth factor
    // Temp data
    double violaPos;
    double violaPosPenaD;
    double violaPosPena;

public:
    StompLegLimitPenalty() = default;

    void setupParams(SwingTrajPlannerConfig config)
    {
        posBd_ << config.joint1PosMin, config.joint1PosMax,
            config.joint2PosMin, config.joint2PosMax,
            config.joint3PosMin, config.joint3PosMax;
        weight_ = config.jointPosWeight;
        mu_ = config.smoothingFactor;
    }

    bool attachPena(const Eigen::Vector3d &pos, double &pena)
    {
        bool outLimitFlag = false;
        for (int idx = 0; idx < 3; idx++)
        {
            // If pos < min_pos
            violaPos = posBd_(idx, 0) - pos(idx);
            if (smoothedL1(violaPos, mu_, violaPosPena, violaPosPenaD))
            {
                pena += weight_ * violaPosPena;
                outLimitFlag = true;
            }
            // If pos > max_pos
            violaPos = pos(idx) - posBd_(idx, 1);
            if (smoothedL1(violaPos, mu_, violaPosPena, violaPosPenaD))
            {
                pena += weight_ * violaPosPena;
                outLimitFlag = true;
            }
        }
        return outLimitFlag;
    }
};