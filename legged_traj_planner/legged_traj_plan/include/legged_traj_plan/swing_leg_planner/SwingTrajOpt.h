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
#include <pinocchio/spatial/se3.hpp>
/* internal project header files */
#include "Utils.h"
#include "LegLimitPenalty.h"
#include "CollisionPenalty.h"

#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

#include "legged_traj_search/geo_utils/lbfgs.hpp"
#include "legged_traj_search/geo_utils/polyhedra.hpp"

class SwingTrajOpt
{

private:
    bool useCfgSpace_;

    ElSpiderAirInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;
    minco::MINCO_S2NU minco;
    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    GCSVisualizer visualizer_ = {nh_, "odom", "swing_traj_opt"};

    // Conditions
    double rho;
    Eigen::Matrix<double, 3, 2> headPV;
    Eigen::Matrix<double, 3, 2> tailPV;
    Eigen::Matrix3Xd polyPath;
    pinocchio::SE3 pose0_;
    pinocchio::SE3 pose1_;
    int index_;

    // Minco Parameters
    Eigen::VectorXi pieceIdx;
    int pieceN;
    int spatialDim;
    int temporalDim;

    // Parameters
    double smoothEps;
    int integralRes;
    Eigen::VectorXd magnitudeBd;
    Eigen::VectorXd penaltyWt;
    Eigen::VectorXd physicalPm;
    double allocSpeed;

    // Penalties
    LegLimitPenalty lmtPena;
    LegCollisionPenalty collPena;

    // Intermediate variables
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

    static inline void forwareP(const Eigen::VectorXd &xi,
                                Eigen::Matrix3Xd &points)
    {
        const int sizeN = xi.size() / 3;
        points.resize(3, sizeN);
        for (int i = 0; i < sizeN; i++)
        {
            points.col(i) = xi.segment<3>(3 * i);
        }
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
    static inline void backwardP(const Eigen::Matrix3Xd &points,
                                 EIGENVEC &xi)
    {
        for (int i = 0; i < points.cols(); i++)
        {
            xi(3 * i) = points(0, i);
            xi(3 * i + 1) = points(1, i);
            xi(3 * i + 2) = points(2, i);
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

    template <typename EIGENVEC>
    static inline void backwardGradP(const Eigen::Matrix3Xd &gradByPoints,
                                     EIGENVEC &gradXi)
    {
        for (int i = 0; i < gradByPoints.cols(); i++)
        {
            gradXi(3 * i) = gradByPoints(0, i);
            gradXi(3 * i + 1) = gradByPoints(1, i);
            gradXi(3 * i + 2) = gradByPoints(2, i);
        }
        return;
    }

    // Soft Constraint

    // TODO:
    /**
     * @brief
     *
     * @param[in] T Time vector
     * @param[in] coeffs Coefficients of trajectory (3, 4 * pieceNum)
     * @param[in] smoothFactor Smooth factor for soft constraint cost function
     * @param[in] integralResolution Integral resolution
     * @param[in] magnitudeBounds [v_max, a_max]^T
     * @param[in] penaltyWeights [pos_weight, vel_weight, acc_weight]^T
     * @param[out] cost Cost
     * @param[out] gradT Gradient of time allocation
     * @param[out] gradC Gradient of coefficients
     */
    static inline void attachPenaltyFunctional(SwingTrajOpt &obj,
                                               const Eigen::VectorXd &T,
                                               const Eigen::MatrixX3d &coeffs,
                                               const double &smoothFactor,
                                               const int &integralResolution,
                                               const Eigen::VectorXd &magnitudeBounds,
                                               const Eigen::VectorXd &penaltyWeights,
                                               double &cost,
                                               Eigen::VectorXd &gradT,
                                               Eigen::MatrixX3d &gradC,
                                               bool verbose = true)
    {
        const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
        const double accSqrMax = magnitudeBounds(1) * magnitudeBounds(1);

        const double weightPos = penaltyWeights(0);
        const double weightVel = penaltyWeights(1);
        const double weightAcc = penaltyWeights(2);

        Eigen::Vector3d pos, vel, acc, jer;
        Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc;
        Eigen::Vector3d gradPos, gradVel, gradAcc;

        double step, alpha;
        double s1, s2, s3;
        Eigen::Matrix<double, 4, 1> beta0, beta1, beta2, beta3;
        Eigen::Vector3d outerNormal;
        int K, L;
        double violaPos, violaVel, violaAcc;
        double violaPosPenaD, violaVelPenaD, violaAccPenaD;
        double violaPosPena, violaVelPena, violaAccPena;
        double node, pena;

        const int pieceNum = T.size();
        const double total_time = T.sum();
        const double integralFrac = 1.0 / integralResolution;
        double time = 0.0;

        // Temp vis
        std::vector<Eigen::Vector3d> visTraj;
        std::vector<Eigen::Vector3d> visTraj2;
        std::vector<Eigen::Vector3d> visInPs;
        obj.collPena.visClear();
        for (int i = 0; i < pieceNum; i++)
        {
            const Eigen::Matrix<double, 4, 3> &c = coeffs.block<4, 3>(i * 4, 0);
            step = T(i) * integralFrac;
            for (int j = 0; j <= integralResolution; j++)
            {
                time += step;
                // Derivatives of traj
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3;
                beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2;
                beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1;
                beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;

                // TODO: Penalties
                gradPos.setZero(), gradVel.setZero(), gradAcc.setZero();
                pena = 0.0;

                double norm_time = time / total_time;
                if (obj.useCfgSpace_)
                {
                    // Joint Limit Soft Constraints
                    obj.lmtPena.attachPena(pos, vel, acc, gradPos, gradVel, gradAcc, pena);
                    if (norm_time > 0.1 && norm_time < 0.9) // Exclude the start and end points
                        obj.collPena.attachPena(poseLinearInterp(obj.pose0_, obj.pose1_, norm_time), pos, gradPos, obj.index_, pena);
                }
                else
                {
                    // Cartesian Space Soft Constraints
                    // TODO
                }

                // Visualizer
                if (obj.useCfgSpace_)
                {
                    visTraj.push_back(point_SE3Act(poseLinearInterp(obj.pose0_, obj.pose1_, norm_time).inverse(), obj.robot_interface_.FK_foot(pos, obj.index_)));
                    visTraj2.push_back(pos);
                    if (j == 0 || j == integralResolution)
                        visInPs.push_back(point_SE3Act(poseLinearInterp(obj.pose0_, obj.pose1_, norm_time).inverse(), obj.robot_interface_.FK_foot(pos, obj.index_)));
                }
                else
                {
                    visTraj.push_back(pos);
                    if (j == 0 || j == integralResolution)
                        visInPs.push_back(pos);
                }

                // Backward
                // std::cout<<"gradPos: "<<gradPos.transpose()<<std::endl;
                // std::cout << "pena: " << pena << std::endl;
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                // pena = 0;
                // totalGradPos = Eigen::Vector3d::Zero();
                // totalGradVel = Eigen::Vector3d::Zero();
                // totalGradAcc = Eigen::Vector3d::Zero();     

                // PROBLEM: What is this
                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                alpha = j * integralFrac;
                gradC.block<4, 3>(i * 4, 0) += (beta0 * totalGradPos.transpose() + // 4*3 order-1 newton?
                                                beta1 * totalGradVel.transpose() + // 4*3
                                                beta2 * totalGradAcc.transpose() *
                                                    node * step);
                // PROBLEM
                gradT(i) += (totalGradPos.dot(vel) +
                             totalGradVel.dot(acc) +
                             totalGradAcc.dot(jer) *
                                 alpha * node * step +
                             node * integralFrac * pena);
                cost += node * step * pena;
            }
        }
        // Visualizer
        obj.visualizer_.delCube();
        obj.visualizer_.delCurve();
        obj.visualizer_.visCurve(visTraj2, ros_visualizer::VisStyle(0.1, 0.8, 0.1, 0.8, 0.005));
        obj.visualizer_.visCurve(visTraj, ros_visualizer::VisStyle(0.1, 0.8, 0.1, 0.8, 0.005));
        obj.visualizer_.visCube(visInPs, Eigen::Vector4d(1, 0, 0, 0), ros_visualizer::VisStyle(0.1, 0.8, 0.1, 1.0, 0.01));
        obj.rate_.sleep();
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

        // Forward
        forwardT(tau, obj.times);
        forwareP(xi, obj.points);

        double cost;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost); // 1.Energy cost
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

        // TODO: 2.Penalty cost
        attachPenaltyFunctional(obj, obj.times, obj.minco.getCoeffs(),
                                obj.smoothEps, obj.integralRes,
                                obj.magnitudeBd, obj.penaltyWt,
                                cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

        // propogate gradient from c,tau to q,t
        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                obj.gradByPoints, obj.gradByTimes);

        // 3.Time cost
        cost += weightT * obj.times.sum();
        obj.gradByTimes.array() += weightT; // PROBLEM

        // Backward
        backwardGradP(obj.gradByPoints, gradXi);
        backwardGradT(tau, obj.gradByTimes, gradTau);

        std::cout << "gradXi: " << gradXi.transpose() << std::endl;
        std::cout << "gradTau: " << gradTau.transpose() << std::endl;

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
                    std::cout << "innerPoint: " << (a + c * m).transpose() << std::endl;
                }
            }
        }
    }

public:
    SwingTrajOpt(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          collPena(robot_interface, gridmap_interface){};

    /**
     * @brief Setup MINCO optimization problem
     *
     * @param TrajPolyPath Config space poly path
     * @param initialVel Initial position, velocity
     * @param terminalVel Terminal position, velocity
     * @param timeWeight
     * @param lengthPerPiece
     * @param smoothingFactor
     * @param integralResolution
     * @param magnitudeBounds [p_max, v_max, a_max]^T
     * @param penaltyWeights [pos_weight, vel_weight, acc_weigh]^T
     * @param physicalParams [?]^T
     * @return true
     * @return false
     */
    inline bool setup(
        // Conditions
        const pinocchio::SE3 &pose0,
        const pinocchio::SE3 &pose1,
        const int &index,
        // Init waypoints
        const Eigen::Matrix3Xd &TrajPolyPath,
        const Eigen::Vector3d &initialVel,
        const Eigen::Vector3d &terminalVel,
        // Params
        const double &timeWeight,
        const double &lengthPerPiece,
        const double &smoothingFactor,
        const int &integralResolution,
        const Eigen::VectorXd &magnitudeBounds,
        const Eigen::VectorXd &penaltyWeights,
        const Eigen::VectorXd &physicalParams,
        // Settings
        const bool useCfgSpace = false,
        const bool verbose = true)
    {
        useCfgSpace_ = useCfgSpace;
        pose0_ = pose0;
        pose1_ = pose1;
        index_ = index;
        if (TrajPolyPath.cols() < 3)
        {
            polyPath.resize(3, 3);
            polyPath.col(0) = TrajPolyPath.col(0);
            polyPath.col(1) = (TrajPolyPath.col(0) + TrajPolyPath.col(1)) / 2.0;
            polyPath.col(2) = TrajPolyPath.col(1);
        }
        else
            polyPath = TrajPolyPath;

        headPV.col(0) = polyPath.leftCols(1);
        headPV.col(1) = initialVel;
        tailPV.col(0) = polyPath.rightCols(1);
        tailPV.col(1) = terminalVel;

        rho = timeWeight;
        smoothEps = smoothingFactor;
        integralRes = integralResolution;
        magnitudeBd = magnitudeBounds;
        penaltyWt = penaltyWeights;
        physicalPm = physicalParams;
        // FIXME: What's this used for?
        allocSpeed = 1.0;

        // subdivide cfg poly path if exceeds length limit
        const Eigen::Matrix3Xd deltas = polyPath.rightCols(polyPath.cols() - 1) -
                                        polyPath.leftCols(polyPath.cols() - 1);
        // FIXME: why innerpoint is added here when piece num =2
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

        // FIXME: ghost variables
        Eigen::Matrix<double, 3, 2> posBd;
        posBd << -0.785, 0.785, -0.5233, 3.14, -0.6978, 3.925;
        Eigen::Vector2d magBd(5, 10);
        Eigen::Vector3d weight(0.1, 0.1, 0.1);
        lmtPena.setup(posBd, magBd, weight, smoothingFactor);

        if (verbose)
        {
            std::cout << "\tPiece num: " << pieceN << std::endl;
            std::cout << "\tSpatial dim: " << spatialDim << std::endl;
            std::cout << "\tTemporal dim: " << temporalDim << std::endl;
        }

        return true;
    }

    inline bool optimize(Trajectory<3> &traj,
                         const double &relCostTol, const bool verbose = true)
    {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        // Set initial values
        setInitial(polyPath, allocSpeed, pieceIdx, points, times);
        backwardT(times, tau); // times to tau
        backwardP(points, xi); // points to xi

        // lbfgs params
        double minCostFunctional;
        lbfgs_params.mem_size = 256;
        lbfgs_params.past = 3;
        lbfgs_params.min_step = 1.0e-32;
        lbfgs_params.g_epsilon = 0.0;
        lbfgs_params.delta = relCostTol;
        // FIXME: TEST
        // lbfgs_params.max_linesearch = 128;

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
            // traj.clear();
            minCostFunctional = INFINITY;
            std::cout << "Optimization Failed: "
                      << lbfgs::lbfgs_strerror(ret)
                      << std::endl;
        }
        return ret >= 0;
        // return minCostFunctional;
    }
};
