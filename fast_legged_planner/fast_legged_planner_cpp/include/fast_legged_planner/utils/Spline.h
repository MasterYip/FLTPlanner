/**
 * @file Spline.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-01-31
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */
#include <cmath>
/* c++ standard library header files */
#include <vector>
#include <iostream>
#include <stdexcept>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */

/**
 * @brief Evaluate cubic spline at t
 *
 * @param para_mat parameter matrix of the spline
 * @param knots spline knots (uniform bspline: 4 knots; cubic hermite: [p0, p1, v0, v1])
 * @param t parameter t
 * @param d_order derivative order (0 for position, 1 for velocity, etc.)
 * @return Eigen::VectorXd evaluated value
 */
Eigen::VectorXd cubic_evaluate(const Eigen::MatrixXd &para_mat, const Eigen::MatrixXd &knots, double t, int d_order = 0)
{
    Eigen::VectorXd result;
    if (d_order > 1)
    {
        throw std::invalid_argument("Invalid derivative order");
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

class SplineBase
{
private:
    Eigen::MatrixXd params_;

public:
    virtual ~SplineBase() {} // Virtual destructor

    // Methods

    /**
     * @brief Evaluate the spline at t.
     *
     * @param t interpolation parameter
     * @param d_order derivative order (0 for position, 1 for velocity, etc.)
     * @param normalized whether to use normalized parameter t
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) = 0;
    virtual void set(const Eigen::MatrixXd &params) = 0;
    virtual Eigen::MatrixXd get() const = 0;
    virtual void insert(double t); // TODO

    // Attributes
    virtual std::pair<double, double> get_range() const = 0;
    virtual int get_dimen() const = 0;
    virtual Eigen::VectorXd get_start() const = 0;
    virtual Eigen::VectorXd get_end() const = 0;
};

/**
 * @brief Uniform B-Spline
 * @details
 * FIXME: Supports only cubic splines for now
 */
class UniBSpline : public SplineBase
{
private:
    int k_ = 3;                         // Order of the spline
    int n;                              // Node count
    int dimen_;                         // Dimension of the spline
    Eigen::MatrixXd params_;            // Parameters of the spline(nodes in rows)
    Eigen::MatrixXd coeff_mat_;         // Coefficient matrix for Cubic Uniform B-Spline
    std::pair<double, double> t_range_; // Range of parameter t

public:
    UniBSpline(const Eigen::MatrixXd &params, int k = 3) : k_(k)
    {
        set(params);
        coeff_mat_ = Eigen::MatrixXd::Zero(k_ + 1, k_ + 1);
        coeff_mat_ << 1, 4, 1, 0,
            -3, 0, 3, 0,
            3, -6, 3, 0,
            -1, 3, -3, 1;
    }

    void set(const Eigen::MatrixXd &params) override
    {
        this->params_ = params;
        n = params.rows();
        dimen_ = params.cols();
        t_range_ = std::make_pair(0, n - 1);
    }

    Eigen::MatrixXd get() const override
    {
        return params_;
    }
    // TODO: test it
    Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) override
    {
        if (normalized)
        {
            t = t * (t_range_.second - t_range_.first) + t_range_.first;
        }

        if (t < t_range_.first || t > t_range_.second)
        {
            throw std::invalid_argument("Parameter t must be in t_range_");
        }

        int i = floor(t);
        Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(k_ + 1, dimen_);
        for (int j = 0; j < k_ + 1; j++)
        {
            knots.row(j) = params_.row(i + j - k_ / 2);
        }
        return cubic_evaluate(coeff_mat_, knots, t - i, d_order);
    }

    std::pair<double, double> get_range() const override
    {
        return t_range_;
    }

    int get_dimen() const override
    {
        return dimen_;
    }

    Eigen::VectorXd get_start() const override
    {
        return params_.row(0);
    }

    Eigen::VectorXd get_end() const override
    {
        return params_.row(n - 1);
    }
};
