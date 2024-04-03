/**
 * @file SwingTrajOpt.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

class SwingTrajOpt
{

private:
    ElSpiderAirInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;

    minco::MINCO_S2NU minco;

    double rho;
    Eigen::Matrix<double, 3, 2> headPV;
    Eigen::Matrix<double, 3, 2> tailPV;

    Eigen::Matrix3Xd polyPath;

    Eigen::VectorXi pieceIdx;

    int pieceN;

    int spatialDim;
    int temporalDim;

    double smoothEps;
    int integralRes;
    Eigen::VectorXd magnitudeBd;
    Eigen::VectorXd penaltyWt;
    Eigen::VectorXd physicalPm;
    double allocSpeed;

    lbfgs::lbfgs_parameter_t lbfgs_params;

    Eigen::Matrix3Xd points;
    Eigen::VectorXd times;
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX3d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;

    // Time domain manifold transformation
    /**
     * @brief tau to T (simplified, exp^tau = T 2-order approximation)
     *
     * @param tau
     * @param T
     */
    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T)
    {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++)
        {
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &T,
                                 EIGENVEC &tau)
    {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++)
        {
            tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                     const Eigen::VectorXd &gradT,
                                     EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;
        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    // Soft Constraint
    template <typename EIGENVEC>
    static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                           const Eigen::VectorXi &vIdx,
                                           const PolyhedraV &vPolys,
                                           double &cost,
                                           EIGENVEC &gradXi)
    {
        const int sizeP = vIdx.size();
        gradXi.resize(xi.size());

        double sqrNormQ, sqrNormViolation, c, dc;
        Eigen::VectorXd q;
        for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
        {
            k = vPolys[vIdx(i)].cols();

            q = xi.segment(j, k);
            sqrNormQ = q.squaredNorm();
            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi.segment(j, k) += dc * 2.0 * q;
            }
        }

        return;
    }

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

    // TODO:
    /**
     * @brief
     *
     * @param T Time vector
     * @param coeffs Coefficients of trajectory (3, 4 * pieceNum)
     * @param smoothFactor Smooth factor for soft constraint cost function
     * @param integralResolution Integral resolution
     * @param magnitudeBounds [v_max, a_max]^T
     * @param penaltyWeights [pos_weight, vel_weight]^T
     * @param cost Cost
     * @param gradT Gradient of time allocation
     * @param gradC Gradient of coefficients
     */
    // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
    //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
    static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                               const Eigen::MatrixX3d &coeffs,
                                               const double &smoothFactor,
                                               const int &integralResolution,
                                               const Eigen::VectorXd &magnitudeBounds,
                                               const Eigen::VectorXd &penaltyWeights,
                                               double &cost,
                                               Eigen::VectorXd &gradT,
                                               Eigen::MatrixX3d &gradC)
    {
        // const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
        // const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
        // const double thetaMax = magnitudeBounds(2);
        // const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
        // const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
        // const double thrustSqrRadi = thrustRadi * thrustRadi;

        // const double weightPos = penaltyWeights(0);
        // const double weightVel = penaltyWeights(1);
        // const double weightOmg = penaltyWeights(2);
        // const double weightTheta = penaltyWeights(3);
        // const double weightThrust = penaltyWeights(4);

        Eigen::Vector3d pos, vel, acc, jer, sna;
        Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
        double totalGradPsi, totalGradPsiD;
        double thr, cos_theta;
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;
        double gradThr;
        Eigen::Vector4d gradQuat;
        Eigen::Vector3d gradPos, gradVel, gradOmg;

        double step, alpha;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 4, 1> beta0, beta1, beta2, beta3, beta4;
        Eigen::Vector3d outerNormal;
        int K, L;
        double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
        double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
        double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
        double node, pena;

        const int pieceNum = T.size();
        const double integralFrac = 1.0 / integralResolution;
        for (int i = 0; i < pieceNum; i++)
        {
            const Eigen::Matrix<double, 4, 3> &c = coeffs.block<4, 3>(i * 4, 0);
            step = T(i) * integralFrac;
            for (int j = 0; j <= integralResolution; j++)
            {
                // Derivatives of traj
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3;
                beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2;
                beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1;
                beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;

                violaVel = vel.squaredNorm() - velSqrMax;
                violaOmg = omg.squaredNorm() - omgSqrMax;
                // 2-order approx of cos(theta)
                cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                violaTheta = acos(cos_theta) - thetaMax;
                violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                gradThr = 0.0;
                gradQuat.setZero();
                gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                pena = 0.0;

                L = hIdx(i);
                K = hPolys[L].rows();
                // Position out of the corridor
                for (int k = 0; k < K; k++)
                {
                    outerNormal = hPolys[L].block<1, 3>(k, 0);
                    violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                    if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                    {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        pena += weightPos * violaPosPena;
                    }
                }

                if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                {
                    gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                    pena += weightVel * violaVelPena;
                }

                if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    pena += weightOmg * violaOmgPena;
                }

                if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
                {
                    gradQuat += weightTheta * violaThetaPenaD /
                                sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                                Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                    pena += weightTheta * violaThetaPena;
                }

                if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
                {
                    gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    pena += weightThrust * violaThrustPena;
                }

                flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);

                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                alpha = j * integralFrac;
                gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                beta1 * totalGradVel.transpose() +
                                                beta2 * totalGradAcc.transpose() +
                                                beta3 * totalGradJer.transpose()) *
                                               node * step;
                gradT(i) += (totalGradPos.dot(vel) +
                             totalGradVel.dot(acc) +
                             totalGradAcc.dot(jer) +
                             totalGradJer.dot(sna)) *
                                alpha * node * step +
                            node * integralFrac * pena;
                cost += node * step * pena;
            }
        }

        return;
    }

    static inline double costFunctional(void *ptr,
                                        const Eigen::VectorXd &x,
                                        Eigen::VectorXd &g)
    {
        SwingTrajOpt &obj = *(SwingTrajOpt *)ptr;
        const int dimTau = obj.temporalDim;
        const int dimXi = obj.spatialDim;
        const double weightT = obj.rho;
        Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
        Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
        Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
        Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

        forwardT(tau, obj.times);

        double cost;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

        // TODO
        attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                obj.smoothEps, obj.integralRes,
                                obj.magnitudeBd, obj.penaltyWt,
                                cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

        // propogate gradient from c,tau to q,t
        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                obj.gradByPoints, obj.gradByTimes);

        cost += weightT * obj.times.sum();
        obj.gradByTimes.array() += weightT;

        backwardGradT(tau, obj.gradByTimes, gradTau);

        // TODO: what is this?
        // normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

        return cost;
    }

    /**
     * @brief Set the Initial waypoints & time allocation
     *
     * @param[in] path Config space poly path
     * @param[in] speed Heuristic speed
     * @param[in] intervalNs Piece num of each poly path segment
     * @param[out] innerPoints New path (Add points that break up path if one segment is too long)
     * @param[out] timeAlloc Time allocation of waypoints
     */
    static inline void setInitial(Eigen::Matrix3Xd &path,
                                  const double &speed,
                                  const Eigen::VectorXi &intervalNs,
                                  Eigen::Matrix3Xd &innerPoints,
                                  Eigen::VectorXd &timeAlloc)
    {
        const int sizeM = intervalNs.size(); // Poly path segment num
        const int sizeN = intervalNs.sum();  // Minco path piece num
        innerPoints.resize(3, sizeN - 1);
        timeAlloc.resize(sizeN);

        Eigen::Vector3d a, b, c;
        for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
        {
            l = intervalNs(i);
            a = path.col(i);
            b = path.col(i + 1);
            c = (b - a) / l;
            timeAlloc.segment(j, l).setConstant(c.norm() / speed);
            j += l;
            for (int m = 0; m < l; m++)
            {
                if (i > 0 || m > 0)
                {
                    innerPoints.col(k++) = a + c * m;
                }
            }
        }
    }

public:
    SwingTrajOpt(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface){};

    /**
     * @brief Setup MINCO optimization problem
     *
     * @param cfgPolyPath Config space poly path
     * @param initialVel Initial position, velocity
     * @param terminalVel Terminal position, velocity
     * @param timeWeight
     * @param lengthPerPiece
     * @param smoothingFactor
     * @param integralResolution
     * @param magnitudeBounds [p_max, v_max, a_max]^T
     * @param penaltyWeights [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
     * @param physicalParams [?]^T
     * @return true
     * @return false
     */
    inline bool setup(
        // Init waypoints
        const Eigen::Matrix3Xd &cfgPolyPath,
        const Eigen::Vector3d &initialVel,
        const Eigen::Vector3d &terminalVel,
        // Params
        const double &timeWeight,
        const double &lengthPerPiece,
        const double &smoothingFactor,
        const int &integralResolution,
        const Eigen::VectorXd &magnitudeBounds,
        const Eigen::VectorXd &penaltyWeights,
        const Eigen::VectorXd &physicalParams)
    {
        polyPath = cfgPolyPath;
        headPV.col(0) = cfgPolyPath.leftCols(1);
        headPV.col(1) = initialVel;
        tailPV.col(0) = cfgPolyPath.rightCols(1);
        tailPV.col(1) = terminalVel;

        rho = timeWeight;
        smoothEps = smoothingFactor;
        integralRes = integralResolution;
        magnitudeBd = magnitudeBounds;
        penaltyWt = penaltyWeights;
        physicalPm = physicalParams;
        // FIXME:
        allocSpeed = magnitudeBd(0) * 3.0;

        // subdivide cfg poly path if exceeds length limit
        const Eigen::Matrix3Xd deltas = cfgPolyPath.rightCols(cfgPolyPath.cols() - 1) -
                                        cfgPolyPath.leftCols(cfgPolyPath.cols() - 1);
        pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
        pieceIdx.array() += 1;
        pieceN = pieceIdx.sum();

        temporalDim = pieceN;
        spatialDim = 3 * (pieceN - 1);

        // Setup for minco
        minco.setConditions(headPV, tailPV, pieceN);

        // Allocate temp variables
        points.resize(3, pieceN - 1);
        times.resize(pieceN);
        gradByPoints.resize(3, pieceN - 1);
        gradByTimes.resize(pieceN);
        partialGradByCoeffs.resize(4 * pieceN, 3); // NOTE:4-order traj
        partialGradByTimes.resize(pieceN);

        return true;
    }

    inline double optimize(Trajectory<3> &traj,
                           const double &relCostTol)
    {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        // Set initial values
        setInitial(polyPath, allocSpeed, pieceIdx, points, times);
        backwardT(times, tau); // times to tau

        // lbfgs params
        double minCostFunctional;
        lbfgs_params.mem_size = 256;
        lbfgs_params.past = 3;
        lbfgs_params.min_step = 1.0e-32;
        lbfgs_params.g_epsilon = 0.0;
        lbfgs_params.delta = relCostTol;

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCostFunctional,
                                        &SwingTrajOpt::costFunctional,
                                        nullptr,
                                        nullptr,
                                        this,
                                        lbfgs_params);

        if (ret >= 0)
        {
            forwardT(tau, times);
            minco.setParameters(points, times);
            minco.getTrajectory(traj);
        }
        else
        {
            traj.clear();
            minCostFunctional = INFINITY;
            std::cout << "Optimization Failed: "
                      << lbfgs::lbfgs_strerror(ret)
                      << std::endl;
        }

        return minCostFunctional;
    }
};
}
;