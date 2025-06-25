/**
 * @file UnitreeA1Interface.h
 * @author GitHub Copilot
 * @brief Unitree A1 Robot Interface
 * @version 0.1
 * @date 2025-06-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <string>
/* external project header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
// FIXME: msgs are ros related
#include <sensor_msgs/JointState.h>
#include "legged_traj_plan/FootState.h"

// ================================
// Unitree A1 Robot Parameters
// ================================

// Link lengths (from A1 URDF specifications)
const double A1_HIP_LINK_LENGTH = 0.0838; // hip ab/ad distance (l1)
const double A1_THIGH_LINK_LENGTH = 0.2;  // thigh link length (l2)
const double A1_CALF_LINK_LENGTH = 0.2;   // calf link length (l3)

// Hip positions in base frame (from A1 robot geometry)
const std::vector<Eigen::Vector3d> A1_HIP_POSITIONS = {
    Eigen::Vector3d(0.1805, -0.047, 0.0),  // FR hip
    Eigen::Vector3d(0.1805, 0.047, 0.0),   // FL hip
    Eigen::Vector3d(-0.1805, -0.047, 0.0), // RR hip
    Eigen::Vector3d(-0.1805, 0.047, 0.0)   // RL hip
};

// Joint limits (radians)
const double A1_HIP_JOINT_MIN = -1.0;   // q1 min
const double A1_HIP_JOINT_MAX = 1.0;    // q1 max
const double A1_THIGH_JOINT_MIN = -1.5; // q2 min
const double A1_THIGH_JOINT_MAX = 3.0;  // q2 max
const double A1_CALF_JOINT_MIN = -2.7;  // q3 min
const double A1_CALF_JOINT_MAX = -0.9;  // q3 max

// Nominal foot positions for A1 (BASE frame)
const double A1_NOMINAL_X = 0.18;  // front/rear distance from center
const double A1_NOMINAL_Y = 0.13;  // left/right distance from center
const double A1_NOMINAL_Z = -0.32; // nominal height

// Define JOINT_STATE_NAME and FOOT_LINK_NAME constants for A1
const std::vector<std::string> A1_JOINT_STATE_NAME = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

const std::vector<std::string> A1_FOOT_LINK_NAME = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

// Nominal foot positions using parameters
const std::vector<Eigen::Vector3d> A1_NOMINAL_FOOT_POS = {
    Eigen::Vector3d(A1_NOMINAL_X, -A1_NOMINAL_Y, A1_NOMINAL_Z),  // FR
    Eigen::Vector3d(A1_NOMINAL_X, A1_NOMINAL_Y, A1_NOMINAL_Z),   // FL
    Eigen::Vector3d(-A1_NOMINAL_X, -A1_NOMINAL_Y, A1_NOMINAL_Z), // RR
    Eigen::Vector3d(-A1_NOMINAL_X, A1_NOMINAL_Y, A1_NOMINAL_Z)   // RL
};

inline Polyhedra genA1LegPolyRegion(int index)
{
    Eigen::Matrix3Xd hull;
    switch (index)
    {
    case 0: // FR
        hull.resize(3, 8);
        hull << 0.25, 0.15, -0.15, -0.25, 0.25, 0.15, -0.15, -0.25,
            -0.15, -0.25, -0.25, -0.15, -0.15, -0.25, -0.25, -0.15,
            -0.05, -0.05, -0.05, -0.05, -0.45, -0.45, -0.45, -0.45;
        break;
    case 1: // FL
        hull.resize(3, 8);
        hull << 0.25, 0.15, -0.15, -0.25, 0.25, 0.15, -0.15, -0.25,
            0.15, 0.25, 0.25, 0.15, 0.15, 0.25, 0.25, 0.15,
            -0.05, -0.05, -0.05, -0.05, -0.45, -0.45, -0.45, -0.45;
        break;
    case 2: // RR
        hull.resize(3, 8);
        hull << -0.25, -0.15, 0.15, 0.25, -0.25, -0.15, 0.15, 0.25,
            -0.15, -0.25, -0.25, -0.15, -0.15, -0.25, -0.25, -0.15,
            -0.05, -0.05, -0.05, -0.05, -0.45, -0.45, -0.45, -0.45;
        break;
    case 3: // RL
        hull.resize(3, 8);
        hull << -0.25, -0.15, 0.15, 0.25, -0.25, -0.15, 0.15, 0.25,
            0.15, 0.25, 0.25, 0.15, 0.15, 0.25, 0.25, 0.15,
            -0.05, -0.05, -0.05, -0.05, -0.45, -0.45, -0.45, -0.45;
        break;
    default:
        throw std::invalid_argument("Invalid leg index for A1 robot (should be 0-3)");
    }
    return Polyhedra(hull);
}

class UnitreeA1Interface : public BaseRobotInterface
{
private:
    std::vector<Eigen::Vector3d> nominal_footholds = {
        Eigen::Vector3d(A1_NOMINAL_X, -A1_NOMINAL_Y, A1_NOMINAL_Z),  // FR
        Eigen::Vector3d(A1_NOMINAL_X, A1_NOMINAL_Y, A1_NOMINAL_Z),   // FL
        Eigen::Vector3d(-A1_NOMINAL_X, -A1_NOMINAL_Y, A1_NOMINAL_Z), // RR
        Eigen::Vector3d(-A1_NOMINAL_X, A1_NOMINAL_Y, A1_NOMINAL_Z)   // RL
    };

public:
    UnitreeA1Interface(const std::string &urdf, const std::vector<std::string> &package_dirs = {})
        : BaseRobotInterface(urdf, package_dirs)
    {
        // Foot convex hull
        for (int i = 0; i < 4; i++)
        {
            foot_polyhedra_.emplace_back(genA1LegPolyRegion(i));
        }
    }

    // Kinematics
    std::vector<double> IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        std::vector<double> q;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Vector3d q_i = IKFast_foot(footendpos[i], i);
            q.push_back(q_i[0]);
            q.push_back(q_i[1]);
            q.push_back(q_i[2]);
        }
        return q;
    }

    Eigen::Vector3d IKFast_foot(const Eigen::Vector3d &footendpos, int index) override
    {
        // Convert from base frame to leg frame with hip shift
        // This matches the transformation used in the ConvexMPC code:
        // pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
        Eigen::Vector3d pDes = footendpos - A1_HIP_POSITIONS[index];

        // Use the same algorithm as computeInverseKinematics in LegController.cpp
        double l1 = A1_HIP_LINK_LENGTH; // ab_ad
        double l2 = A1_THIGH_LINK_LENGTH;
        double l3 = A1_CALF_LINK_LENGTH;

        double q1, q2, q3;
        double b2y, b3z, b4z, a, b, c;
        int sideSign = 1; // 1 for Left legs; -1 for right legs
        if (index == 0 || index == 2)
        {
            sideSign = -1;
        }

        b2y = l1 * sideSign;
        b3z = -l2;
        b4z = -l3;
        a = l1;
        c = sqrt(pow(pDes[0], 2) + pow(pDes[1], 2) + pow(pDes[2], 2)); // whole length
        b = sqrt(pow(c, 2) - pow(a, 2));                               // distance between shoulder and footpoint

        // q1_ik implementation
        double L = sqrt(pow(pDes[1], 2) + pow(pDes[2], 2) - pow(b2y, 2));
        q1 = atan2(pDes[2] * b2y + pDes[1] * L, pDes[1] * b2y - pDes[2] * L);

        // q3_ik implementation
        double temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
        if (temp > 1)
            temp = 1;
        if (temp < -1)
            temp = -1;
        q3 = acos(temp);
        q3 = -(M_PI - q3); // 0~180

        // q2_ik implementation
        double a1 = pDes[1] * sin(q1) - pDes[2] * cos(q1);
        double a2 = pDes[0];
        double m1 = b4z * sin(q3);
        double m2 = b3z + b4z * cos(q3);
        q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

        return Eigen::Vector3d(q1, q2, q3);
    }

    // Overload for constraint checking with boolean return
    bool IKFast_foot(const Eigen::Vector3d &footendpos, Eigen::Vector3d &q_result, int index) override
    {
        bool check_constraints = true;

        // Convert from base frame to leg frame with hip shift
        Eigen::Vector3d pDes = footendpos - A1_HIP_POSITIONS[index];

        // Use the same algorithm as computeInverseKinematics in LegController.cpp
        double l1 = A1_HIP_LINK_LENGTH; // ab_ad
        double l2 = A1_THIGH_LINK_LENGTH;
        double l3 = A1_CALF_LINK_LENGTH;

        double q1, q2, q3;
        double b2y, b3z, b4z, a, b, c;
        int sideSign = 1; // 1 for Left legs; -1 for right legs
        if (index == 0 || index == 2)
        {
            sideSign = -1;
        }

        b2y = l1 * sideSign;
        b3z = -l2;
        b4z = -l3;
        a = l1;
        c = sqrt(pow(pDes[0], 2) + pow(pDes[1], 2) + pow(pDes[2], 2)); // whole length

        if (check_constraints)
        {
            // Check basic reachability constraints
            double max_reach = l2 + l3;
            double min_reach = abs(l2 - l3);

            if (c > max_reach || c < a) // c must be at least hip length
            {
                return false;
            }

            // Check if hip offset calculation is valid
            if (c * c - a * a < 0)
            {
                return false;
            }
        }

        try
        {
            b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

            // q1_ik implementation
            double temp_hip = pow(pDes[1], 2) + pow(pDes[2], 2) - pow(b2y, 2);
            if (temp_hip < 0)
                return false; // Invalid hip calculation

            double L = sqrt(temp_hip);
            q1 = atan2(pDes[2] * b2y + pDes[1] * L, pDes[1] * b2y - pDes[2] * L);

            // q3_ik implementation
            double temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
            if (temp > 1)
                temp = 1;
            if (temp < -1)
                temp = -1;
            q3 = acos(temp);
            q3 = -(M_PI - q3); // 0~180

            // q2_ik implementation
            double a1 = pDes[1] * sin(q1) - pDes[2] * cos(q1);
            double a2 = pDes[0];
            double m1 = b4z * sin(q3);
            double m2 = b3z + b4z * cos(q3);
            q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

            if (check_constraints)
            {
                // Additional joint limit checks using parameters
                if (q1 < A1_HIP_JOINT_MIN || q1 > A1_HIP_JOINT_MAX ||
                    q2 < A1_THIGH_JOINT_MIN || q2 > A1_THIGH_JOINT_MAX ||
                    q3 < A1_CALF_JOINT_MIN || q3 > A1_CALF_JOINT_MAX)
                {
                    return false;
                }
            }

            q_result = Eigen::Vector3d(q1, q2, q3);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index) override
    {
        // Determine side sign
        int sideSign = (index == 0 || index == 2) ? -1 : 1;

        double s1 = sin(q[0]);
        double s2 = sin(q[1]);
        double s3 = sin(q[2]);
        double c1 = cos(q[0]);
        double c2 = cos(q[1]);
        double c3 = cos(q[2]);

        double c23 = c2 * c3 - s2 * s3;
        double s23 = s2 * c3 + c2 * s3;

        // Forward kinematics equations in leg frame (same as LegController.cpp)
        double px = -A1_CALF_LINK_LENGTH * s23 - A1_THIGH_LINK_LENGTH * s2;
        double py = A1_HIP_LINK_LENGTH * sideSign * c1 + A1_CALF_LINK_LENGTH * (s1 * c23) + A1_THIGH_LINK_LENGTH * c2 * s1;
        double pz = A1_HIP_LINK_LENGTH * sideSign * s1 - A1_CALF_LINK_LENGTH * (c1 * c23) - A1_THIGH_LINK_LENGTH * c1 * c2;

        // Convert from leg frame to base frame
        Eigen::Vector3d pLeg(px, py, pz);
        return pLeg + A1_HIP_POSITIONS[index];
    }

    Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index) override
    {
        int sideSign = (index == 0 || index == 2) ? -1 : 1;

        double s1 = sin(q[0]);
        double s2 = sin(q[1]);
        double s3 = sin(q[2]);
        double c1 = cos(q[0]);
        double c2 = cos(q[1]);
        double c3 = cos(q[2]);

        double c23 = c2 * c3 - s2 * s3;
        double s23 = s2 * c3 + c2 * s3;

        Eigen::Matrix3Xd J(3, 3);

        // Jacobian matrix elements (same as LegController.cpp)
        J(0, 0) = 0;
        J(1, 0) = -sideSign * A1_HIP_LINK_LENGTH * s1 + A1_THIGH_LINK_LENGTH * c2 * c1 + A1_CALF_LINK_LENGTH * c23 * c1;
        J(2, 0) = sideSign * A1_HIP_LINK_LENGTH * c1 + A1_THIGH_LINK_LENGTH * c2 * s1 + A1_CALF_LINK_LENGTH * c23 * s1;

        J(0, 1) = -A1_CALF_LINK_LENGTH * c23 - A1_THIGH_LINK_LENGTH * c2;
        J(1, 1) = -A1_THIGH_LINK_LENGTH * s2 * s1 - A1_CALF_LINK_LENGTH * s23 * s1;
        J(2, 1) = A1_THIGH_LINK_LENGTH * s2 * c1 + A1_CALF_LINK_LENGTH * s23 * c1;

        J(0, 2) = -A1_CALF_LINK_LENGTH * c23;
        J(1, 2) = -A1_CALF_LINK_LENGTH * s23 * s1;
        J(2, 2) = A1_CALF_LINK_LENGTH * s23 * c1;

        // Note: This Jacobian is in leg frame. For base frame operations,
        // the calling code should account for hip offsets if needed
        return J;
    }

    Eigen::Matrix3Xd getJacobianTimeVariation(const Eigen::Vector3d &q, const Eigen::Vector3d &vel, int index) override
    {
        // Numerical approximation of Jacobian time derivative
        // For a more accurate implementation, analytical derivatives would be computed
        const double dt = 1e-6;
        Eigen::Matrix3Xd J_current = getJacobian(q, index);
        Eigen::Matrix3Xd J_next = getJacobian(q + vel * dt, index);
        return (J_next - J_current) / dt;
    }

    // Simple collision ball forward kinematics (for compatibility)
    Eigen::Vector3d FK_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx) override
    {
        // For A1, we'll approximate collision balls at joint positions
        int sideSign = (legIdx == 0 || legIdx == 2) ? -1 : 1;

        double s1 = sin(q[0]);
        double s2 = sin(q[1]);
        double c1 = cos(q[0]);
        double c2 = cos(q[1]);

        Eigen::Vector3d pLeg;
        switch (jointIdx)
        {
        case 0: // Hip joint position
            pLeg = Eigen::Vector3d(0, A1_HIP_LINK_LENGTH * sideSign * c1, A1_HIP_LINK_LENGTH * sideSign * s1);
            break;
        case 1: // Knee joint position
            pLeg = Eigen::Vector3d(-A1_THIGH_LINK_LENGTH * s2,
                                   A1_HIP_LINK_LENGTH * sideSign * c1 + A1_THIGH_LINK_LENGTH * c2 * s1,
                                   A1_HIP_LINK_LENGTH * sideSign * s1 - A1_THIGH_LINK_LENGTH * c1 * c2);
            break;
        case 2: // Foot position
            return FK_foot(q, legIdx);
        default:
            pLeg = Eigen::Vector3d::Zero();
        }

        // Convert from leg frame to base frame
        return pLeg + A1_HIP_POSITIONS[legIdx];
    }

    Eigen::Matrix3Xd getJacobian_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx) override
    {
        // Simplified Jacobian for collision balls
        if (jointIdx == 2)
        {
            return getJacobian(q, legIdx);
        }
        else
        {
            // For intermediate joints, return a simplified Jacobian
            // This would need proper implementation based on specific joint
            return getJacobian(q, legIdx);
        }
    }

    Eigen::Matrix3d getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q, const Eigen::Vector3d &vel,
                                                      int legIdx, int jointIdx) override
    {
        // Return 3x3 matrix instead of 3xN for compatibility
        Eigen::Matrix3Xd J_dot = getJacobianTimeVariation(q, vel, legIdx);
        return J_dot.block<3, 3>(0, 0);
    }

    // Get nominal foothold positions
    Eigen::Vector3d getNominalFoothold(int index) override
    {
        if (index >= 0 && index < 4)
        {
            return nominal_footholds[index];
        }
        throw std::out_of_range("Invalid leg index for getNominalFoothold");
    }

    // Feedback Interface
    // Foot state in BASE frame
    const legged_traj_plan::FootState &getFootStateFdb() const override
    {
        throw std::runtime_error("Not implemented");
    }

    const sensor_msgs::JointState &getJointStateFdb() const override
    {
        throw std::runtime_error("Not implemented");
    }

    const pinocchio::SE3 &getBodyPoseFdb() const override
    {
        throw std::runtime_error("Not implemented");
    }

    const pinocchio::Motion &getBodyVelFdb() const override
    {
        throw std::runtime_error("Not implemented");
    }

    // Command Interface

    /**
     * @brief Set body pose (for dummy robot)
     *
     * @param body_pose
     */
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setBodyVelCmd(const pinocchio::Motion &body_vel) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setJointCmd(const std::vector<double> &q) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact) override
    {
        throw std::runtime_error("Not implemented");
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                     const std::vector<Eigen::Vector3d> &v,
                     const std::vector<Eigen::Vector3d> &tau,
                     const std::vector<bool> &contact) override
    {
        throw std::runtime_error("Not implemented");
    }
};