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
#include "fast_legged_planner/utils/Trajectory.h"

////////////////////
// Consts

const Eigen::MatrixXd UNIB_COE_MAT = (Eigen::MatrixXd(4, 4) << 1, 4, 1, 0,
                                      -3, 0, 3, 0,
                                      3, -6, 3, 0,
                                      -1, 3, -3, 1)
                                         .finished() /
                                     6;

/**
 * @brief Evaluate cubic spline at t
 *
 * @param para_mat parameter matrix of the spline
 * @param knots spline knots (uniform bspline: 4 knots; cubic hermite: [p0, p1, v0, v1])
 * @param t parameter t
 * @param d_order derivative order (0 for position, 1 for velocity, etc.)
 * @return Eigen::VectorXd evaluated value
 */
Eigen::VectorXd cubic_evaluate(const Eigen::MatrixXd &para_mat, const Eigen::MatrixXd &knots, double t, int d_order = 0);

class SplineBase : public TrajectoryBase
{
private:
    Eigen::MatrixXd params_;

public:
    // Methods

    /**
     * @brief Evaluate the spline at t.
     *
     * @param t interpolation parameter
     * @param d_order derivative order (0 for position, 1 for velocity, etc.)
     * @param normalized whether to use normalized parameter t
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false)
    {
        printf("Warning: evaluate() not implemented in derived class\n");
        return Eigen::VectorXd::Zero(get_dimen());
    };
    /**
     * @brief Set decision variables of the spline
     *
     * @param params decision variables
     */
    virtual void set(const Eigen::MatrixXd &params)
    {
        params_ = params;
    };
    /**
     * @brief Get decision variables of the spline
     *
     * @return Eigen::MatrixXd
     */
    virtual Eigen::MatrixXd get() const
    {
        return params_;
    };
    // virtual void insert(double t); // TODO

    // Attributes
    virtual std::pair<double, double> get_range() const
    {
        printf("Warning: get_range() not implemented in derived class\n");
        return std::make_pair(0, 0);
    };
    virtual int get_dimen() const
    {
        printf("Warning: get_dimen() not implemented in derived class\n");
        return 1;
    };
    virtual Eigen::VectorXd get_start() const
    {
        printf("Warning: get_start() not implemented in derived class\n");
        return Eigen::VectorXd::Zero(get_dimen());
    };
    virtual Eigen::VectorXd get_end() const
    {
        printf("Warning: get_end() not implemented in derived class\n");
        return Eigen::VectorXd::Zero(get_dimen());
    };
};

/**
 * @brief Uniform B-Spline
 * @details
 * FIXME: Supports only cubic splines for now
 */
class UniBSpline : public SplineBase
{
private:
    int k_ = 3;                                // Order of the spline
    int n;                                     // Node count
    int dimen_;                                // Dimension of the spline
    Eigen::MatrixXd params_;                   // Parameters of the spline(nodes in rows)
    Eigen::MatrixXd coeff_mat_ = UNIB_COE_MAT; // Coefficient matrix for Cubic Uniform B-Spline
    std::pair<double, double> t_range_;        // Range of parameter t

public:
    // FIXME: add default constructor
    UniBSpline(){};

    UniBSpline(const Eigen::MatrixXd &params, int k = 3) : k_(k)
    {
        set(params);
    }

    void set(const Eigen::MatrixXd &params) override
    {
        this->params_ = params;
        n = params.rows();
        dimen_ = params.cols();
        // t_range_ = std::make_pair(0, n - 1);
        // NOTE: in order to reach the start/end of the spline, we need to extend the t_range
        t_range_ = std::make_pair(2 - k_, n + k_ - 3);
    }

    Eigen::MatrixXd get() const override
    {
        return params_;
    }
    // TODO: test it
    Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) override
    {
        // convert normalized t to real t
        if (normalized && t >= 0.0 && t <= 1.0)
        {
            t = t * (t_range_.second - t_range_.first) + t_range_.first;
        }

        if (t < t_range_.first || t > t_range_.second)
        {
            std::cerr << "Parameter t out of range" << std::endl;
            // Saturation
            if (t < t_range_.first)
                t = t_range_.first;
            if (t > t_range_.second)
                t = t_range_.second;
        }

        int i = floor(t);
        Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(k_ + 1, dimen_);
        // FIXME: this evaluate method can't reach the start/end of the spline
        for (int j = 0; j < k_ + 1; j++)
        {
            int index = i + j - k_ / 2;
            // Clamp index to [0, n-1]
            if (index < 0)
                index = 0;
            if (index > n - 1)
                index = n - 1;
            knots.row(j) = params_.row(index);
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
