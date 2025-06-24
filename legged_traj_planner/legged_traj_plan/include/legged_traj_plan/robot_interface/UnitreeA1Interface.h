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

// Define JOINT_STATE_NAME and FOOT_LINK_NAME constants for A1
const std::vector<std::string> A1_JOINT_STATE_NAME = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

const std::vector<std::string> A1_FOOT_LINK_NAME = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

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

// Nominal foot positions for A1 (BASE frame)
const double a1_nominal_x = 0.18;  // front/rear distance from center
const double a1_nominal_y = 0.13;  // left/right distance from center
const double a1_nominal_z = -0.32; // nominal height
const std::vector<Eigen::Vector3d> A1_NOMINAL_FOOT_POS = {
    Eigen::Vector3d(a1_nominal_x, -a1_nominal_y, a1_nominal_z),  // FR
    Eigen::Vector3d(a1_nominal_x, a1_nominal_y, a1_nominal_z),   // FL
    Eigen::Vector3d(-a1_nominal_x, -a1_nominal_y, a1_nominal_z), // RR
    Eigen::Vector3d(-a1_nominal_x, a1_nominal_y, a1_nominal_z)   // RL
};

class UnitreeA1Interface : public BaseRobotInterface
{
private:
    std::vector<Eigen::Vector3d> nominal_footholds = {
        Eigen::Vector3d(a1_nominal_x, -a1_nominal_y, a1_nominal_z),  // FR
        Eigen::Vector3d(a1_nominal_x, a1_nominal_y, a1_nominal_z),   // FL
        Eigen::Vector3d(-a1_nominal_x, -a1_nominal_y, a1_nominal_z), // RR
        Eigen::Vector3d(-a1_nominal_x, a1_nominal_y, a1_nominal_z)   // RL
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
        // A1 link lengths (from URDF or robot specifications)
        const double l1 = 0.0838; // hip link length (ab/ad distance)
        const double l2 = 0.2;    // thigh link length
        const double l3 = 0.2;    // calf link length

        // Hip positions in base frame (from A1 robot geometry)
        std::vector<Eigen::Vector3d> hip_positions = {
            Eigen::Vector3d(0.1805, -0.047, 0.0),   // FR hip
            Eigen::Vector3d(0.1805, 0.047, 0.0),    // FL hip  
            Eigen::Vector3d(-0.1805, -0.047, 0.0),  // RR hip
            Eigen::Vector3d(-0.1805, 0.047, 0.0)    // RL hip
        };

        // Convert from base frame to leg frame
        Eigen::Vector3d pDes = footendpos - hip_positions[index];
        
        // Determine side sign: -1 for right legs (FR, RR), +1 for left legs (FL, RL)
        int sideSign = (index == 0 || index == 2) ? -1 : 1; // FR=0, FL=1, RR=2, RL=3

        double px = pDes[0];
        double py = pDes[1]; 
        double pz = pDes[2];

        // Use the same IK algorithm as in LegController.cpp
        double c = sqrt(px*px + py*py + pz*pz);  // whole length
        double b = sqrt(c*c - l1*l1);  // distance between shoulder and footpoint

        // Hip joint angle (q1) - same as q1_ik in LegController.cpp
        double L = sqrt(py*py + pz*pz - l1*l1);
        double q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);

        // Knee joint angle (q3) - same as q3_ik in LegController.cpp  
        double temp = (l2*l2 + l3*l3 - b*b) / (2.0 * l2 * l3);
        temp = std::max(-1.0, std::min(1.0, temp)); // clamp to valid range
        double q3 = acos(temp);
        q3 = -(M_PI - q3); // A1 convention: negative knee angle

        // Thigh joint angle (q2) - same as q2_ik in LegController.cpp
        double a1 = py * sin(q1) - pz * cos(q1);
        double a2 = px;
        double m1 = l3 * sin(q3);
        double m2 = l2 + l3 * cos(q3);
        double q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

        return Eigen::Vector3d(q1, q2, q3);
    }

    // Overload for constraint checking with boolean return
    bool IKFast_foot(const Eigen::Vector3d &footendpos, Eigen::Vector3d &q_result, int index) override
    {
        bool check_constraints = true;
        // A1 link lengths
        const double l1 = 0.0838;
        const double l2 = 0.2;
        const double l3 = 0.2;

        // Hip positions in base frame
        std::vector<Eigen::Vector3d> hip_positions = {
            Eigen::Vector3d(0.1805, -0.047, 0.0),   // FR hip
            Eigen::Vector3d(0.1805, 0.047, 0.0),    // FL hip  
            Eigen::Vector3d(-0.1805, -0.047, 0.0),  // RR hip
            Eigen::Vector3d(-0.1805, 0.047, 0.0)    // RL hip
        };

        // Convert from base frame to leg frame
        Eigen::Vector3d pDes = footendpos - hip_positions[index];
        
        double px = pDes[0];
        double py = pDes[1];
        double pz = pDes[2];

        // Check if point is reachable (basic constraint checking)
        double c = sqrt(px*px + py*py + pz*pz);
        double max_reach = l2 + l3;
        double min_reach = abs(l2 - l3);

        if (check_constraints)
        {
            // Check basic reachability constraints
            if (c > max_reach || c < min_reach)
            {
                return false;
            }

            // Check if hip offset is reachable
            double hip_distance = sqrt(py*py + pz*pz);
            if (hip_distance < l1)
            {
                return false;
            }
        }

        try
        {
            // Use same algorithm as non-constraint version
            double b = sqrt(c*c - l1*l1);
            
            double L = sqrt(py*py + pz*pz - l1*l1);
            if (L != L) // Check for NaN
                return false;

            double q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);

            double temp = (l2*l2 + l3*l3 - b*b) / (2.0 * l2 * l3);
            temp = std::max(-1.0, std::min(1.0, temp));
            double q3 = acos(temp);
            q3 = -(M_PI - q3);

            double a1 = py * sin(q1) - pz * cos(q1);
            double a2 = px;
            double m1 = l3 * sin(q3);
            double m2 = l2 + l3 * cos(q3);
            double q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

            if (check_constraints)
            {
                // Additional joint limit checks
                const double q1_min = -1.0, q1_max = 1.0;
                const double q2_min = -1.5, q2_max = 3.0;
                const double q3_min = -2.7, q3_max = -0.9;

                if (q1 < q1_min || q1 > q1_max ||
                    q2 < q2_min || q2 > q2_max ||
                    q3 < q3_min || q3 > q3_max)
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
        // A1 link lengths
        const double l1 = 0.0838; // hip link length
        const double l2 = 0.2;    // thigh link length
        const double l3 = 0.2;    // calf link length

        // Hip positions in base frame
        std::vector<Eigen::Vector3d> hip_positions = {
            Eigen::Vector3d(0.1805, -0.047, 0.0),   // FR hip
            Eigen::Vector3d(0.1805, 0.047, 0.0),    // FL hip  
            Eigen::Vector3d(-0.1805, -0.047, 0.0),  // RR hip
            Eigen::Vector3d(-0.1805, 0.047, 0.0)    // RL hip
        };

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
        double px = -l3 * s23 - l2 * s2;
        double py = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
        double pz = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;

        // Convert from leg frame to base frame
        Eigen::Vector3d pLeg(px, py, pz);
        return pLeg + hip_positions[index];
    }

    Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index) override
    {
        // A1 link lengths
        const double l1 = 0.0838;
        const double l2 = 0.2;
        const double l3 = 0.2;

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
        J(1, 0) = -sideSign * l1 * s1 + l2 * c2 * c1 + l3 * c23 * c1;
        J(2, 0) = sideSign * l1 * c1 + l2 * c2 * s1 + l3 * c23 * s1;

        J(0, 1) = -l3 * c23 - l2 * c2;
        J(1, 1) = -l2 * s2 * s1 - l3 * s23 * s1;
        J(2, 1) = l2 * s2 * c1 + l3 * s23 * c1;

        J(0, 2) = -l3 * c23;
        J(1, 2) = -l3 * s23 * s1;
        J(2, 2) = l3 * s23 * c1;

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
        // Hip positions in base frame
        std::vector<Eigen::Vector3d> hip_positions = {
            Eigen::Vector3d(0.1805, -0.047, 0.0),   // FR hip
            Eigen::Vector3d(0.1805, 0.047, 0.0),    // FL hip  
            Eigen::Vector3d(-0.1805, -0.047, 0.0),  // RR hip
            Eigen::Vector3d(-0.1805, 0.047, 0.0)    // RL hip
        };

        // For A1, we'll approximate collision balls at joint positions
        const double l1 = 0.0838;
        const double l2 = 0.2;

        int sideSign = (legIdx == 0 || legIdx == 2) ? -1 : 1;

        double s1 = sin(q[0]);
        double s2 = sin(q[1]);
        double c1 = cos(q[0]);
        double c2 = cos(q[1]);

        Eigen::Vector3d pLeg;
        switch (jointIdx)
        {
        case 0: // Hip joint position
            pLeg = Eigen::Vector3d(0, l1 * sideSign * c1, l1 * sideSign * s1);
            break;
        case 1: // Knee joint position
            pLeg = Eigen::Vector3d(-l2 * s2,
                                   l1 * sideSign * c1 + l2 * c2 * s1,
                                   l1 * sideSign * s1 - l2 * c1 * c2);
            break;
        case 2: // Foot position
            return FK_foot(q, legIdx);
        default:
            pLeg = Eigen::Vector3d::Zero();
        }
        
        // Convert from leg frame to base frame
        return pLeg + hip_positions[legIdx];
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

    Eigen::Vector3d getNominalFoothold(int index) override
    {
        // Return nominal footholds in base frame (already correct)
        return nominal_footholds[index];
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