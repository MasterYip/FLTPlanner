/**
 * @file PrimitivePenalty.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-14
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
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/Utils.h"

class PrimitivePenalty
{
private:
    // Config
    Eigen::Vector3d primitivePos_;    // [min_pos, max_pos]
    Eigen::Vector3d primitiveWeight_; // [weight_pos, weight_vel, weight_acc]
    // Temp data
    double violaPos;
    double violaPosPenaD;
    double violaPosPena;

public:
    PrimitivePenalty() = default;

    void setup(SwingTrajPlannerConfig &config)
    {
        primitivePos_ << config.primitivePos1, config.primitivePos2, config.primitivePos3;
        primitiveWeight_ << config.primitiveWeight1, config.primitiveWeight2, config.primitiveWeight3;
    }

    /**
     * @brief Attach L2 penalty to position error
     *
     * @param pos Position in config space
     * @param gradPos Gradient of position in config space
     * @param pena Penalty
     */
    void attachPena(const Eigen::Vector3d &pos, Eigen::Vector3d &gradPos, double &pena)
    {
        for (int idx = 0; idx < 3; idx++)
        {
            violaPos = pos[idx] - primitivePos_[idx];
            penaltyL2(violaPos, violaPosPena, violaPosPenaD);
            gradPos[idx] += primitiveWeight_[idx] * violaPosPenaD;
            pena += primitiveWeight_[idx] * violaPosPena;
        }
    }
};