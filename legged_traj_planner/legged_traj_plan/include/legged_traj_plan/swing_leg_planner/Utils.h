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
#include <pinocchio/spatial/se3.hpp>
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

inline pinocchio::SE3 poseLinearInterp(pinocchio::SE3 pose0, pinocchio::SE3 pose1, double t)
{
    pinocchio::Motion err = pinocchio::log6(pose0.actInv(pose1));
    pinocchio::SE3 interp = pose0.act(pinocchio::exp6(err * t));
    return interp;
}