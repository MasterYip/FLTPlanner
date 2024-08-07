/**
 * @file LegLimitPenalty.h
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
#include "Utils.h"

class LegLimitPenalty
{
private:
    // Config
    Eigen::Matrix<double, 3, 2> posBd_; // [min_pos, max_pos]
    Eigen::Vector2d magnitudeBd_;       // [max_vel, max_acc]
    Eigen::Vector3d weight_;            // [weight_pos, weight_vel, weight_acc]
    double velSqrMax_;
    double accSqrMax_;
    double mu_; // Smooth factor
    // Temp data
    double violaPos, violaVel, violaAcc;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD;
    double violaPosPena, violaVelPena, violaAccPena;

public:
    LegLimitPenalty() = default;

    void setup(const Eigen::Matrix<double, 3, 2> posBd,
               const Eigen::Vector2d &magnitudeBd,
               const Eigen::Vector3d &weight, double mu)
    {
        posBd_ = posBd;
        magnitudeBd_ = magnitudeBd;
        weight_ = weight;
        velSqrMax_ = magnitudeBd_(0) * magnitudeBd_(0);
        accSqrMax_ = magnitudeBd_(1) * magnitudeBd_(1);
        mu_ = mu;
    }

    // TODO: torque
    /**
     * @brief Attach penalty to position, velocity and acceleration
     *
     * @param pos Position in config space
     * @param vel Velocity in config space
     * @param acc Acceleration in config space
     * @param gradPos Gradient of position in config space
     * @param gradVel Gradient of velocity in config space
     * @param gradAcc Gradient of acceleration in config space
     * @param pena Penalty
     */
    void attachPena(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                    Eigen::Vector3d &gradPos, Eigen::Vector3d &gradVel, Eigen::Vector3d &gradAcc,
                    double &pena)
    {
        for (int idx = 0; idx < 3; idx++)
        {
            // If pos < min_pos
            violaPos = posBd_(idx, 0) - pos(idx);
            if (smoothedL1(violaPos, mu_, violaPosPena, violaPosPenaD))
            {
                gradPos(idx) -= weight_(0) * violaPosPenaD;
                pena += weight_(0) * violaPosPena;
            }
            // If pos > max_pos
            violaPos = pos(idx) - posBd_(idx, 1);
            if (smoothedL1(violaPos, mu_, violaPosPena, violaPosPenaD))
            {
                gradPos(idx) += weight_(0) * violaPosPenaD;
                pena += weight_(0) * violaPosPena;
            }

            violaVel = vel(idx) * vel(idx) - velSqrMax_;
            if (smoothedL1(violaVel, mu_, violaVelPena, violaVelPenaD))
            {
                gradVel(idx) += weight_(1) * violaVelPenaD * 2.0 * vel(idx);
                pena += weight_(1) * violaVelPena;
            }
            violaAcc = acc(idx) * acc(idx) - accSqrMax_;
            if (smoothedL1(violaAcc, mu_, violaAccPena, violaAccPenaD))
            {
                gradAcc(idx) += weight_(2) * violaAccPenaD * 2.0 * acc(idx);
                pena += weight_(2) * violaAccPena;
            }
        }
    }
};