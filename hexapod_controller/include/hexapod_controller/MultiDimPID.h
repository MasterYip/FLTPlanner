/**
 * @file multi_dim_PID.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-01-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <Eigen/Dense>

class MultiDimPID
{
  public:
    MultiDimPID(uint dim, double Kp, double Ki, double Kd, double tau, double limMax,
                double limMaxInt, double T)
        : dim_(dim), Kp_(Kp), Ki_(Ki), Kd_(Kd), tau_(tau), limMax_(limMax), limMaxInt_(limMaxInt),
          T_(T)
    {
        integrator_.resize(dim_);
        prevError_.resize(dim_);
        differentiator_.resize(dim_);
        prevMeasurement_.resize(dim_);
        out_.resize(dim_);

        integrator_.setZero();
        prevError_.setZero();
        differentiator_.setZero();
        prevMeasurement_.setZero();
        out_.setZero();
    }
    ~MultiDimPID(){};
    Eigen::VectorXd update(const Eigen::VectorXd &cmd, const Eigen::VectorXd &meas)
    {
        Eigen::VectorXd error = cmd - meas;
        Eigen::VectorXd proportional = Kp_ * error;

        integrator_ = integrator_ + 0.5 * Ki_ * T_ * (error + prevError_);
        // integrator limit
        if (integrator_.norm() > limMaxInt_)
        {
            integrator_ = limMaxInt_ * integrator_.normalized();
        }

        differentiator_ =
            -(2.0 * Kd_ * (meas - prevMeasurement_) + (2.0 * tau_ - T_) * differentiator_) /
            (2.0 * tau_ + T_);

        out_ = proportional + integrator_ + differentiator_;
        // out limit
        if (out_.norm() > limMax_)
        {
            out_ = limMax_ * out_.normalized();
        }

        prevError_ = error;
        prevMeasurement_ = meas;

        return out_;
    };

    Eigen::VectorXd update(const Eigen::VectorXd &cmd, const Eigen::VectorXd &meas, double dt)
    {
        T_ = dt;
        // TODO: temp solution
        tau_ = dt / 2;
        return update(cmd, meas);
    }

  private:
    /* Controller gains */
    uint dim_;

    double Kp_;
    double Ki_;
    double Kd_;

    /* Derivative low-pass filter time constant
    (when tau=T/2, it is classic differentiator) */
    double tau_;

    /* Output limits */
    double limMax_;

    /* Integrator limits */
    double limMaxInt_;

    /* Sample time (in seconds) */
    double T_;

    /* Controller "memory" */
    Eigen::VectorXd integrator_;      /* trapezoid integration */
    Eigen::VectorXd prevError_;       /* Required for integrator */
    Eigen::VectorXd differentiator_;  /* Derivative (band-limited differentiator)
                                       * Z transfer function: D/Y = -2Kd(z-1)/[(2tau+T)z + (2tau-T)]
                                       * When tau = T/2, the differentiator is -Kd/T*(1-z^-1) */
    Eigen::VectorXd prevMeasurement_; /* Required for differentiator */

    /* Controller output */
    Eigen::VectorXd out_;
};
