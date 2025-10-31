/**
 * @file Hexapod201InterfaceROS.h
 * @author Master Yip (2205929492@qq.com)
 * @brief Hexapod201 interface that communicates with hexapod201_interface.py through ROS topics
 * @version 0.1
 * @date 2024-12-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
#include <vector>
#include <string>

/* external project header files */
#include <pinocchio/math/rpy.hpp>
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "legged_traj_plan/FootState.h"

/* internal project header files */

struct Hexapod201InterfaceROSConfig
{
    std::string urdfParamPath;
    std::string urdf; // Auto loaded

    // Topic names for communication with Python interface
    std::string poseCommandTopic;      // Command topic to send pose commands
    std::string pathCommandTopic;      // Command topic to send path commands
    std::string footCommandTopic;      // Command topic to send foot commands
    std::string currentPoseTopic;      // Feedback topic to receive current pose
    std::string footStateTopic;        // Feedback topic to receive foot states
    
    // Frame names
    std::string odomChildFrame;
    std::string odomParentFrame;

    // Init parameters
    std::vector<double> nominalFootPos;      // size 18: (xyz in base frame) * 6
    std::vector<double> initBodyPose;        // size 6: (xyzrpy)

    void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/urdfParamPath", urdfParamPath);
        check_digit &= nh.getParam(urdfParamPath, urdf);
        
        // Topic configuration
        nh.param(ns + "/poseCommandTopic", poseCommandTopic, std::string("/hexapod/pose_cmd"));
        nh.param(ns + "/pathCommandTopic", pathCommandTopic,
                 std::string("/hexapod/path_cmd"));
        nh.param(ns + "/footCommandTopic", footCommandTopic, std::string("/hexapod/foot_cmd"));
        nh.param(ns + "/currentPoseTopic", currentPoseTopic, std::string("/hexapod/current_pose"));
        nh.param(ns + "/footStateTopic", footStateTopic, std::string("/hexapod/foot_state"));
        
        check_digit &= nh.getParam(ns + "/odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam(ns + "/odomParentFrame", odomParentFrame);
        check_digit &= nh.getParam(ns + "/nominalFootPos", nominalFootPos);
        check_digit &= nominalFootPos.size() == 18;
        check_digit &= nh.getParam(ns + "/initBodyPose", initBodyPose);
        check_digit &= initBodyPose.size() == 6;
        
        if (!check_digit)
        {
            ROS_ERROR("Failed to load Hexapod201InterfaceROSConfig");
        }
    }
};

class Hexapod201InterfaceROS : public BaseRobotInterface
{
private:
    ros::NodeHandle nh_;
    Hexapod201InterfaceROSConfig config_;

    // ROS Publishers (Commands to Python interface)
    ros::Publisher pose_cmd_pub_, path_cmd_pub_;
    ros::Publisher foot_cmd_pub_;

    // ROS Subscribers (Feedback from Python interface)
    ros::Subscriber current_pose_sub_;
    ros::Subscriber foot_state_sub_;

    // Internal state storage
    pinocchio::SE3 body_pose_;
    pinocchio::Motion body_vel_;
    legged_traj_plan::FootState foot_state_;
    sensor_msgs::JointState joint_state_;
    std::vector<Eigen::Vector3d> nominal_footholds_;

    // Feedback flags
    bool pose_received_;
    bool foot_state_received_;
    ros::Time last_pose_time_;
    ros::Time last_foot_state_time_;

    // Joint names for hexapod (6 legs * 3 joints per leg)
    std::vector<std::string> JOINT_STATE_NAME_ = {
        "leg0_j0", "leg0_j1", "leg0_j2",
        "leg1_j0", "leg1_j1", "leg1_j2",
        "leg2_j0", "leg2_j1", "leg2_j2",
        "leg3_j0", "leg3_j1", "leg3_j2",
        "leg4_j0", "leg4_j1", "leg4_j2",
        "leg5_j0", "leg5_j1", "leg5_j2"
    };

    // Callback functions
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void footStateCallback(const legged_traj_plan::FootState::ConstPtr& msg);

public:
    explicit Hexapod201InterfaceROS(const Hexapod201InterfaceROSConfig &config);

    //// Overrides - Kinematics Interface (Dummy implementations)
    std::vector<double> IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos) override;
    Eigen::Vector3d IKFast_foot(const Eigen::Vector3d &footendpos, int index) override;
    bool IKFast_foot(const Eigen::Vector3d &footendpos, Eigen::Vector3d &q_result, int index) override;
    Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index) override;
    Eigen::Vector3d FK_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx) override;
    Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index) override;
    Eigen::Matrix3Xd getJacobianTimeVariation(const Eigen::Vector3d &q, const Eigen::Vector3d &vel, int index) override;
    Eigen::Matrix3Xd getJacobian_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx) override;
    Eigen::Matrix3d getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q, const Eigen::Vector3d &vel,
                                                      int legIdx, int jointIdx) override;
    Eigen::Vector3d getNominalFoothold(int index) override;

    //// Overrides - Feedback Interface
    const legged_traj_plan::FootState &getFootStateFdb() const override;
    const sensor_msgs::JointState &getJointStateFdb() const override;
    const pinocchio::SE3 &getBodyPoseFdb() const override;
    const pinocchio::Motion &getBodyVelFdb() const override;

    //// Overrides - Command Interface (Send commands to Python interface)
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override;
    void setStepBodyPathCmd(const nav_msgs::Path &body_path) override;
    void setBodyVelCmd(const pinocchio::Motion &body_vel) override;
    void setJointCmd(const std::vector<double> &q) override;
    void setJointCmd(const std::vector<Eigen::Vector3d> &q) override;
    void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact) override;
    void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                     const std::vector<Eigen::Vector3d> &v,
                     const std::vector<Eigen::Vector3d> &tau,
                     const std::vector<bool> &contact) override;
    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override;
    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact) override;

    // Special function for coordinated body pose and foot commands
    void setStepCmd(const pinocchio::SE3 &body_pose,
                    const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<bool> &contact);

    // Utility functions
    bool isConnected() const;
    double getConnectionTimeout() const { return 2.0; } // 2 seconds timeout
    std::vector<Eigen::Vector3d> getNominalFootholds() const;
};