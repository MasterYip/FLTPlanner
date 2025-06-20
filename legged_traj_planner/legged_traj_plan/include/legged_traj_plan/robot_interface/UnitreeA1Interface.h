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

    // Simple inverse kinematics using pinocchio (no IKFast for A1)
    Eigen::Vector3d IK_foot(const Eigen::Vector3d &footendpos, int index)
    {
        // For A1, we'll use a simple geometric IK solution
        // This is a placeholder - you may want to implement proper IK or use numerical methods
        const std::string foot_frame = A1_FOOT_LINK_NAME[index];
        
        // Use pinocchio's inverse kinematics solver or implement geometric solution
        // For now, return nominal joint configuration
        Eigen::Vector3d q_nominal(0.0, 0.9, -1.8); // Hip, thigh, calf angles
        return q_nominal;
    }

    Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index)
    {
        // Forward kinematics using pinocchio
        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(12); // 4 legs * 3 joints each
        q_full.segment<3>(index * 3) = q;
        
        const std::string foot_frame = A1_FOOT_LINK_NAME[index];
        pinocchio::SE3 foot_pose = get_frame_placement(q_full, foot_frame);
        return foot_pose.translation();
    }

    Eigen::Vector3d getNominalFoothold(int index)
    {
        return nominal_footholds[index];
    }

    // Feedback Interface
    // Foot state in BASE frame
    virtual const legged_traj_plan::FootState &getFootStateFdb() const
    {
        throw std::runtime_error("Not implemented");
    }
    virtual const sensor_msgs::JointState &getJointStateFdb() const
    {
        throw std::runtime_error("Not implemented");
    }
    virtual const pinocchio::SE3 &getBodyPoseFdb() const
    {
        throw std::runtime_error("Not implemented");
    }
    virtual const pinocchio::Motion &getBodyVelFdb() const
    {
        throw std::runtime_error("Not implemented");
    }

    // Command Interface

    /**
     * @brief Set body pose (for dummy robot)
     *
     * @param body_pose
     */
    virtual void setBodyPoseCmd(const pinocchio::SE3 &body_pose)
    {
        throw std::runtime_error("Not implemented");
    }

    virtual void setBodyVelCmd(const pinocchio::Motion &body_vel)
    {
        throw std::runtime_error("Not implemented");
    }

    virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos)
    {
        throw std::runtime_error("Not implemented");
    }

    virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                            const std::vector<Eigen::Vector3d> &footendvel,
                            const std::vector<Eigen::Vector3d> &footendeffort,
                            const std::vector<bool> &contact)
    {
        throw std::runtime_error("Not implemented");
    }

    virtual void setJointCmd(const std::vector<double> &q)
    {
        throw std::runtime_error("Not implemented");
    }
    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q)
    {
        throw std::runtime_error("Not implemented");
    }
    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact)
    {
        throw std::runtime_error("Not implemented");
    }
    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q, 
                             const std::vector<Eigen::Vector3d> &v,
                             const std::vector<Eigen::Vector3d> &tau,
                             const std::vector<bool> &contact)
    {
        throw std::runtime_error("Not implemented");
    }
};