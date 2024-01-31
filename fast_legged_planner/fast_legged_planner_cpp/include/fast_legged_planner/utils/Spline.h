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

#include <Eigen/Dense>
#include <vector>
#include <iostream>

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
    int k_ = 3; // Order of the spline
    int n;
    int dimen_;
    Eigen::MatrixXd params_;
    std::pair<double, double> t_range_;
    // You need to define interpolate.BSpline or equivalent in C++ for the bspline member.

public:
    UniBSpline(const Eigen::MatrixXd &params, int k = 3) : k_(k)
    {
        set(params);
    }

    void set(const Eigen::MatrixXd &params) override
    {
        this->params_ = params;
        n = params.rows();
        dimen_ = params.cols();
        t_range_ = std::make_pair(0, n - 1);

        // You need to initialize bspline here using the provided parameters.
        // This part depends on how you implement the interpolation in C++.
    }

    Eigen::MatrixXd get() const override
    {
        return params_;
    }

    Eigen::VectorXd evaluate(double t, int d_order = 0, bool normalized = false) override
    {
        // You need to implement the spline evaluation logic here.
        // Use the interpolation mechanism you have in C++.
        std::cout << "Evaluate function called." << std::endl;
        return Eigen::VectorXd(); // Placeholder return value
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
