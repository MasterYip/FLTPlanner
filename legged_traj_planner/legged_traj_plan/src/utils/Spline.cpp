/**
 * @file Spline.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "legged_traj_plan/utils/Spline.h"

Eigen::VectorXd cubic_evaluate(const Eigen::MatrixXd &para_mat, const Eigen::MatrixXd &knots, double t, int d_order)
{
    Eigen::VectorXd result;
    if (d_order > 1)
    {
        throw std::invalid_argument("Invalid derivative order: " + std::to_string(d_order));
    }
    if (t < 0 || t > 1)
    {
        throw std::invalid_argument("Parameter t must be in range [0, 1]");
    }
    if (d_order == 0)
    {
        Eigen::VectorXd t_vec(4);
        t_vec << 1, t, pow(t, 2), pow(t, 3);
        result = t_vec.transpose() * para_mat * knots;
    }
    else if (d_order == 1)
    {
        Eigen::VectorXd t_vec(4);
        t_vec << 0, 1, 2 * t, 3 * pow(t, 2);
        result = t_vec.transpose() * para_mat * knots;
    }
    return result;
}