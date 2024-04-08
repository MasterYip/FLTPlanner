/**
 * @file MotorLimitPenalty.h
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

class MotorLimitPenalty
{
private:
    Eigen::VectorXd magnitudeBd_(5); // [min_pos, max_pos, max_vel, max_acc, max_torque]
    double mu_;                      // Smooth factor
public:
    MotorLimitPenalty(const Eigen::VectorXd &magnitudeBd, double mu)
    {
        if (magnitudeBd.size() != 5)
        {
            throw std::runtime_error("magnitudeBd should have 5 elements");
        }
        magnitudeBd_ = magnitudeBd;
        mu_ = mu;
    }

    // TODO: torque
    void attachPena(double pos, double vel, double acc,
                    double &pena, double &dpena)
    {
    }
};