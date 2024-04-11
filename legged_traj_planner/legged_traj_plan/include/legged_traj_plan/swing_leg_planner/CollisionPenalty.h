/**
 * @file CollisionPenalty.h
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

/* external project header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"

class LegCollisionPenalty
{
private:
    ElSpiderAirInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;
    Eigen::Vector3d collBallRadius_;
    Eigen::Vector3d weight_;
    double mu_;

public:
    LegCollisionPenalty(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface)
    {
        // TODO: use setup()
        collBallRadius_ << 0.12, 0.12, 0.5;
        weight_ << 1.0, 1.0, 1.0;
        mu_ = 0.01;
    }

    /**
     * @brief Attach penalty to position, velocity and acceleration
     *
     * @param pose Pose of base
     * @param pos Position in config space
     * @param gradPos Gradient of position in config space
     * @param pena Penalty
     */
    void attachPena(const pinocchio::SE3 &pose,
                    const Eigen::Vector3d &posCfg,
                    Eigen::Vector3d &gradPosCfg,
                    int index,
                    double &pena)
    {
        // WORLD frame
        Eigen::Vector3d footPos = robot_interface_.FK_foot(posCfg, index);
        Eigen::Vector3d sdfGrad;
        Eigen::Vector3d gradPos;

        double sdf = gridmap_interface_.sdfValue(footPos, 0, "min");
        double f, df;
        if (smoothedL1(collBallRadius_(2) - sdf, mu_, f, df))
        {
            sdfGrad = gridmap_interface_.sdfDerivative(footPos, 0);
            gradPos = df * sdfGrad / sdfGrad.norm();
            Eigen::Matrix3Xd J = robot_interface_.getJacobian(posCfg, index);
            Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
            // std::cout << "J_inv: " << J_inv << std::endl;
            gradPosCfg += weight_(2) * J_inv * gradPos;
            pena += weight_(2) * f;
        }
    }
};