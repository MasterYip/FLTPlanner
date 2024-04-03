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

    static inline double costFunctional(void *ptr,
                                        const Eigen::VectorXd &x,
                                        Eigen::VectorXd &g)
    {
        GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
        const int dimTau = obj.temporalDim;
        const int dimXi = obj.spatialDim;
        const double weightT = obj.rho;
        Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
        Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
        Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
        Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

        forwardT(tau, obj.times);
        forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

        double cost;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

        attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                obj.hPolyIdx, obj.hPolytopes,
                                obj.smoothEps, obj.integralRes,
                                obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

        // propogate gradient from c,tau to q,t
        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                obj.gradByPoints, obj.gradByTimes);

        cost += weightT * obj.times.sum();
        obj.gradByTimes.array() += weightT;

        backwardGradT(tau, obj.gradByTimes, gradTau);
        backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
        normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

        return cost;
    }

public:
    SwingTrajOpt(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface){};

    /**
     * @brief Setup MINCO optimization problem
     *
     * @param timeWeight
     * @param initialPVA Initial position, velocity
     * @param terminalPVA Terminal position, velocity
     * @param safeCorridor H-rep polys
     * @param lengthPerPiece
     * @param smoothingFactor
     * @param integralResolution
     * @param magnitudeBounds [p_max, v_max, a_max]^T
     * @param penaltyWeights [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
     * @param physicalParams [?]^T
     * @return true
     * @return false
     */
    inline bool setup(const double &timeWeight,
                      const Eigen::Matrix3d &initialPV,
                      const Eigen::Matrix3d &terminalPV,
                      const PolyCorridor &Corridor,
                      const double &lengthPerPiece,
                      const double &smoothingFactor,
                      const int &integralResolution,
                      const Eigen::VectorXd &magnitudeBounds,
                      const Eigen::VectorXd &penaltyWeights,
                      const Eigen::VectorXd &physicalParams)
    {
        rho = timeWeight;
        headPVA = initialPVA;
        tailPVA = terminalPVA;

        hPolytopes = safeCorridor;
        for (size_t i = 0; i < hPolytopes.size(); i++)
        {
            // Normalize the normal vectors
            const Eigen::ArrayXd norms =
                hPolytopes[i].leftCols<3>().rowwise().norm();
            hPolytopes[i].array().colwise() /= norms;
        }
        // Process the corridor - H-rep to V-rep + V-rep Intersections
        if (!processCorridor(hPolytopes, vPolytopes))
        {
            return false;
        }

        polyN = hPolytopes.size();
        smoothEps = smoothingFactor;
        integralRes = integralResolution;
        magnitudeBd = magnitudeBounds;
        penaltyWt = penaltyWeights;
        physicalPm = physicalParams;
        allocSpeed = magnitudeBd(0) * 3.0;
        // Get ShortestPath using L-BFGS (polyN+1 points)
        getShortestPath(headPVA.col(0), tailPVA.col(0),
                        vPolytopes, smoothEps, shortPath);
        const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
        pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
        pieceIdx.array() += 1;
        pieceN = pieceIdx.sum();

        temporalDim = pieceN;
        spatialDim = 0;
        vPolyIdx.resize(pieceN - 1);
        hPolyIdx.resize(pieceN);
        for (int i = 0, j = 0, k; i < polyN; i++)
        {
            k = pieceIdx(i);
            for (int l = 0; l < k; l++, j++)
            {
                if (l < k - 1)
                {
                    vPolyIdx(j) = 2 * i;
                    spatialDim += vPolytopes[2 * i].cols();
                }
                else if (i < polyN - 1)
                {
                    vPolyIdx(j) = 2 * i + 1;
                    spatialDim += vPolytopes[2 * i + 1].cols();
                }
                hPolyIdx(j) = i;
            }
        }

        // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
        minco.setConditions(headPVA, tailPVA, pieceN);
        flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                      physicalPm(3), physicalPm(4), physicalPm(5));

        // Allocate temp variables
        points.resize(3, pieceN - 1);
        times.resize(pieceN);
        gradByPoints.resize(3, pieceN - 1);
        gradByTimes.resize(pieceN);
        partialGradByCoeffs.resize(6 * pieceN, 3);
        partialGradByTimes.resize(pieceN);

        return true;
    }

    inline double optimize(Trajectory<5> &traj,
                           const double &relCostTol)
    {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        setInitial(shortPath, allocSpeed, pieceIdx, points, times);
        backwardT(times, tau);
        backwardP(points, vPolyIdx, vPolytopes, xi);

        double minCostFunctional;
        lbfgs_params.mem_size = 256;
        lbfgs_params.past = 3;
        lbfgs_params.min_step = 1.0e-32;
        lbfgs_params.g_epsilon = 0.0;
        lbfgs_params.delta = relCostTol;

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCostFunctional,
                                        &GCOPTER_PolytopeSFC::costFunctional,
                                        nullptr,
                                        nullptr,
                                        this,
                                        lbfgs_params);

        if (ret >= 0)
        {
            forwardT(tau, times);
            forwardP(xi, vPolyIdx, vPolytopes, points);
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