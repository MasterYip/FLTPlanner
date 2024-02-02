/**
 * @file Trajectory.h
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

/* external project header files */
#include <Eigen/Dense>
/* internal project header files */

class TrajectoryBase
{
public:
    /**
     * @brief Evaluate the trajectory at t.
     *
     * @param t normalized interpolation parameter
     * @param d_order derivative order (0 for position, 1 for velocity, etc.)
     * @param normalized whether to use normalized parameter t
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) = 0;
};