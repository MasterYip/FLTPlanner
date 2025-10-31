/**
 * @file DummyHexapod201InterfaceROS.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/JointCmd.h"
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"
#include "ros_visualizer/ros_visualizer.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pinocchio/math/rpy.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/JointCmd.h"
#include "ros_visualizer/ros_visualizer.hpp"
#include <tf2_ros/transform_broadcaster.h>

/* internal project header files */

struct DummyHexapod201InterfaceROSConfig {
  std::string urdfParamPath;
  std::string urdf; // Auto loaded

  std::string jointStateTopic;
  std::string jointNamePrefix;
  std::string odomChildFrame;
  std::string odomParentFrame;
  bool enableVis;
  bool usePyInterface;

  // Init
  std::vector<double> nominalFootPos;      // size 18: (xyz in base frame) * 6
  std::vector<double> nominalFootPosShift; // size 3: (dx dy dz)
  std::vector<double> initBodyPose;        // size 6: (xyzrpy)

  void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface") {
    bool check_digit = true;
    check_digit &= nh.getParam(ns + "/urdfParamPath", urdfParamPath);
    check_digit &= nh.getParam(urdfParamPath, urdf);
    check_digit &= nh.getParam(ns + "/jointStateTopic", jointStateTopic);
    check_digit &= nh.getParam(ns + "/jointNamePrefix", jointNamePrefix);
    check_digit &= nh.getParam(ns + "/odomChildFrame", odomChildFrame);
    check_digit &= nh.getParam(ns + "/odomParentFrame", odomParentFrame);
    check_digit &= nh.getParam(ns + "/enableVis", enableVis);
    check_digit &= nh.getParam(ns + "/usePyInterface", usePyInterface);
    check_digit &= nh.getParam(ns + "/nominalFootPos", nominalFootPos);
    check_digit &= nominalFootPos.size() == 18;
    check_digit &=
        nh.getParam(ns + "/nominalFootPosShift", nominalFootPosShift);
    check_digit &= nominalFootPosShift.size() == 3;
    check_digit &= nh.getParam(ns + "/initBodyPose", initBodyPose);
    check_digit &= initBodyPose.size() == 6;
    if (!check_digit) {
      ROS_ERROR("Failed to load DummyHexapod201InterfaceROSConfig");
    }
  }
};

class DummyHexapod201InterfaceROS : public BaseRobotInterface {
private:
  ros::NodeHandle nh;
  DummyHexapod201InterfaceROSConfig config_;

    // ROS Publishers
    ros::Publisher joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;
    std::shared_ptr<ros_visualizer::ROSVisualizer> visualizer_;
    ros::Publisher pose_cmd_pub, path_cmd_pub;
    ros::Publisher footcmd_pub; // FootState publisher for communication with Python interface

  // states
  legged_traj_plan::FootState foot_state_;
  sensor_msgs::JointState joint_state_;
  pinocchio::SE3 body_pose_;
  pinocchio::Motion body_vel_;
  std::vector<Eigen::Vector3d> nominal_footholds;

  // Joint names for hexapod (6 legs * 3 joints per leg)
  std::vector<std::string> JOINT_STATE_NAME = {
      "leg0_j0", "leg0_j1", "leg0_j2", "leg1_j0", "leg1_j1", "leg1_j2",
      "leg2_j0", "leg2_j1", "leg2_j2", "leg3_j0", "leg3_j1", "leg3_j2",
      "leg4_j0", "leg4_j1", "leg4_j2", "leg5_j0", "leg5_j1", "leg5_j2"};

  // Visualization helpers
  void pub_odom(const pinocchio::SE3 &odom) {
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = config_.odomParentFrame;
    odom_tf.child_frame_id = config_.odomChildFrame;
    odom_tf.transform.translation.x = odom.translation()[0];
    odom_tf.transform.translation.y = odom.translation()[1];
    odom_tf.transform.translation.z = odom.translation()[2];
    Eigen::Quaterniond quat(odom.rotation());
    odom_tf.transform.rotation.x = quat.x();
    odom_tf.transform.rotation.y = quat.y();
    odom_tf.transform.rotation.z = quat.z();
    odom_tf.transform.rotation.w = quat.w();
    odom_pub.sendTransform(odom_tf);
  }

  void pub_joint_state(const std::vector<double> &q) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = JOINT_STATE_NAME;
    if (config_.jointNamePrefix != "") {
      for (auto &name : joint_state.name)
        name = config_.jointNamePrefix + name;
    }
    joint_state.position = q;
    joint_state_pub.publish(joint_state);
  }

  void pub_joint_state(const std::vector<Eigen::Vector3d> &q) {
    std::vector<double> q_vec;
    for (auto pos : q) {
      q_vec.push_back(pos[0]);
      q_vec.push_back(pos[1]);
      q_vec.push_back(pos[2]);
    }
    pub_joint_state(q_vec);
  }

  void vis_foot_positions(const std::vector<Eigen::Vector3d> &footendpos) {
    if (!config_.enableVis || !visualizer_)
      return;

    // Visualize feet as spheres
    visualizer_->delGroup(0); // Clear previous group
    visualizer_->setIdGroup(0);
    visualizer_->visSphere(footendpos, 0.03,
                           ros_visualizer::VisStyle(1.0, 0.0, 0.0, 1.0, 0.03));
  }

  void vis_body_pose(const pinocchio::SE3 &body_pose) {
    if (!config_.enableVis || !visualizer_)
      return;

    // Visualize body as cube
    // visualizer_->delGroup(1); // Clear previous group
    visualizer_->setIdGroup(1);
    Eigen::Vector3d body_pos(0, 0, 0);
    Eigen::Quaterniond quat(0, 0, 0, 1); // Identity quaternion
    Eigen::Vector4d quat_vec(quat.w(), quat.x(), quat.y(), quat.z());
    visualizer_->visCube(
        body_pos, quat_vec,
        ros_visualizer::VisStyle(0.0, 1.0, 0.0, 0.8, 0.3, 0.2, 0.1));
  }

public:
    DummyHexapod201InterfaceROS(const DummyHexapod201InterfaceROSConfig &config)
        : BaseRobotInterface(config.urdf), config_(config)
    {
        // Initialize ROS publishers
        joint_state_pub = nh.advertise<sensor_msgs::JointState>(config_.jointStateTopic, 10);
        if (config_.usePyInterface)
        {
            pose_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/hexapod/pose_cmd", 10);
            footcmd_pub = nh.advertise<legged_traj_plan::FootState>("/hexapod/foot_cmd", 10); // FootState publisher
            path_cmd_pub =
                nh.advertise<nav_msgs::Path>("/hexapod/path_cmd", 10);
        }
        if (config_.enableVis && !config_.usePyInterface)
        {
            visualizer_ = std::make_shared<ros_visualizer::ROSVisualizer>(nh, "base", "hexapod201_markers");
        }

    // Init State
    nominal_footholds.clear();
    foot_state_.position.clear();
    foot_state_.velocity.clear();
    foot_state_.effort.clear();

    for (int i = 0; i < 6; i++) {
      geometry_msgs::Point pt;
      pt.x = config_.nominalFootPos[3 * i] + config_.nominalFootPosShift[0];
      if (i < 3)
        pt.y =
            config_.nominalFootPos[3 * i + 1] - config_.nominalFootPosShift[1];
      else
        pt.y =
            config_.nominalFootPos[3 * i + 1] + config_.nominalFootPosShift[1];
      pt.z = config_.nominalFootPos[3 * i + 2] + config_.nominalFootPosShift[2];
      foot_state_.position.emplace_back(pt);

      // Update nominal foot position
      nominal_footholds.emplace_back(Eigen::Vector3d(pt.x, pt.y, pt.z));

      // Initialize joint positions (dummy values since we don't have real
      // kinematics)
      joint_state_.position.emplace_back(0.0); // j0
      joint_state_.position.emplace_back(0.0); // j1
      joint_state_.position.emplace_back(0.0); // j2
    }

    foot_state_.velocity.resize(6);
    foot_state_.effort.resize(6);
    foot_state_.contact = {true, true, true, true, true, true};

    joint_state_.velocity.resize(18, 0.0);
    joint_state_.effort.resize(18, 0.0);
    joint_state_.name = JOINT_STATE_NAME;

    body_pose_ = pinocchio::SE3(
        pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(config_.initBodyPose[3],
                                                    config_.initBodyPose[4],
                                                    config_.initBodyPose[5])),
        Eigen::Vector3d(config_.initBodyPose[0], config_.initBodyPose[1],
                        config_.initBodyPose[2]));
  }

  //// Overrides - Kinematics Interface (Dummy implementations)
  std::vector<double>
  IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos) override {
    // Dummy implementation - just return zeros since we don't have real
    // kinematics
    return std::vector<double>(18, 0.0);
  }

  Eigen::Vector3d IKFast_foot(const Eigen::Vector3d &footendpos,
                              int index) override {
    // Dummy implementation - return zero joint angles
    return Eigen::Vector3d::Zero();
  }

  bool IKFast_foot(const Eigen::Vector3d &footendpos, Eigen::Vector3d &q_result,
                   int index) override {
    // Dummy implementation - always successful with zero joint angles
    q_result = Eigen::Vector3d::Zero();
    return true;
  }

  Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index) override {
    // Dummy implementation - just return nominal foot position
    if (index >= 0 && index < nominal_footholds.size())
      return nominal_footholds[index];
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d FK_CollBall(const Eigen::Vector3d &q, int legIdx,
                              int jointIdx) override {
    // Dummy implementation
    return Eigen::Vector3d::Zero();
  }

  Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index) override {
    // Dummy implementation - return 3x3 identity matrix
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3Xd getJacobianTimeVariation(const Eigen::Vector3d &q,
                                            const Eigen::Vector3d &vel,
                                            int index) override {
    // Dummy implementation - return zero matrix
    return Eigen::Matrix3d::Zero();
  }

  Eigen::Matrix3Xd getJacobian_CollBall(const Eigen::Vector3d &q, int legIdx,
                                        int jointIdx) override {
    // Dummy implementation
    return Eigen::Matrix3d::Zero();
  }

  Eigen::Matrix3d getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q,
                                                    const Eigen::Vector3d &vel,
                                                    int legIdx,
                                                    int jointIdx) override {
    // Dummy implementation
    return Eigen::Matrix3d::Zero();
  }

  Eigen::Vector3d getNominalFoothold(int index) override {
    if (index >= 0 && index < nominal_footholds.size())
      return nominal_footholds[index];
    return Eigen::Vector3d::Zero();
  }

  //// Overrides - Feedback Interface
  const legged_traj_plan::FootState &getFootStateFdb() const override {
    return foot_state_;
  }

  const sensor_msgs::JointState &getJointStateFdb() const override {
    return joint_state_;
  }

  const pinocchio::SE3 &getBodyPoseFdb() const override { return body_pose_; }

  const pinocchio::Motion &getBodyVelFdb() const override { return body_vel_; }

    //// Overrides - Command Interface
    // Not used in Hexapod201 python interface
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override
    {
        // if (!config_.usePyInterface)
        {
            body_pose_ = body_pose;
            if (config_.enableVis && visualizer_)
            {
                pub_odom(body_pose);
                vis_body_pose(body_pose);
            }
        }
    }

    void setBodyVelCmd(const pinocchio::Motion &body_vel) override
    {
        if (!config_.usePyInterface)
        {
            body_vel_ = body_vel;
        }
    }

    void setStepBodyPathCmd(const nav_msgs::Path &body_path) override {
      path_cmd_pub.publish(body_path);
      // if (config_.usePyInterface) {
      //   path_cmd_pub.publish(body_path);
      // }
      // For non-Python interface, just set the body pose
      // setBodyPoseCmd(body_pose);
    }

  void setJointCmd(const std::vector<double> &q) override {
    if (q.size() >= 18) {
      joint_state_.position = q;
      if (config_.enableVis)
        pub_joint_state(q);
    }
  }

  void setJointCmd(const std::vector<Eigen::Vector3d> &q) override {
    joint_state_.position.clear();
    for (const auto &pos : q) {
      joint_state_.position.emplace_back(pos[0]);
      joint_state_.position.emplace_back(pos[1]);
      joint_state_.position.emplace_back(pos[2]);
    }
    if (config_.enableVis)
      pub_joint_state(joint_state_.position);
  }

  void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                   const std::vector<bool> &contact) override {
    setJointCmd(q);
  }

  void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                   const std::vector<Eigen::Vector3d> &v,
                   const std::vector<Eigen::Vector3d> &tau,
                   const std::vector<bool> &contact) override {
    setJointCmd(q, contact);
  }

    // Used in Hexapod201 python interface
    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        if (config_.usePyInterface)
        {
            legged_traj_plan::FootState footstate;
            footstate.header.stamp = ros::Time::now();

            // Add foot names
            for (int i = 0; i < 6; ++i)
            {
                footstate.name.push_back("foot_" + std::to_string(i));
            }

            for (int i = 0; i < 6 && i < footendpos.size(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = footendpos[i][0];
                pt.y = footendpos[i][1];
                pt.z = footendpos[i][2];
                footstate.position.push_back(pt);

                // Initialize velocity and effort with zeros
                geometry_msgs::Vector3 vec3;
                vec3.x = vec3.y = vec3.z = 0.0;
                footstate.velocity.push_back(vec3);
                footstate.effort.push_back(vec3);

                // Default contact state (true = in contact)
                footstate.contact.push_back(true);
            }
            footcmd_pub.publish(footstate);
        }
        else
        {
            for (int i = 0; i < 6 && i < footendpos.size(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = footendpos[i][0];
                pt.y = footendpos[i][1];
                pt.z = footendpos[i][2];
                foot_state_.position[i] = pt;
            }

            if (config_.enableVis)
            {
                vis_foot_positions(footendpos);
            }
        }
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact) override
    {
        if (config_.usePyInterface)
        {
            legged_traj_plan::FootState footstate;
            footstate.header.stamp = ros::Time::now();

            // Add foot names
            for (int i = 0; i < 6; ++i)
            {
                footstate.name.push_back("foot_" + std::to_string(i));
            }

            for (int i = 0; i < 6 && i < footendpos.size(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = footendpos[i][0];
                pt.y = footendpos[i][1];
                pt.z = footendpos[i][2];
                footstate.position.push_back(pt);

                geometry_msgs::Vector3 vel;
                vel.x = (i < footendvel.size()) ? footendvel[i][0] : 0.0;
                vel.y = (i < footendvel.size()) ? footendvel[i][1] : 0.0;
                vel.z = (i < footendvel.size()) ? footendvel[i][2] : 0.0;
                footstate.velocity.push_back(vel);

                geometry_msgs::Vector3 effort;
                effort.x = (i < footendeffort.size()) ? footendeffort[i][0] : 0.0;
                effort.y = (i < footendeffort.size()) ? footendeffort[i][1] : 0.0;
                effort.z = (i < footendeffort.size()) ? footendeffort[i][2] : 0.0;
                footstate.effort.push_back(effort);

                // Contact state
                bool contact_state = (i < contact.size()) ? contact[i] : true;
                footstate.contact.push_back(contact_state);
            }
            footcmd_pub.publish(footstate);
        }
        else
        {
            setFootCmd(footendpos);

            for (int i = 0; i < 6; ++i)
            {
                if (i < footendvel.size())
                {
                    foot_state_.velocity[i].x = footendvel[i][0];
                    foot_state_.velocity[i].y = footendvel[i][1];
                    foot_state_.velocity[i].z = footendvel[i][2];
                }
                if (i < footendeffort.size())
                {
                    foot_state_.effort[i].x = footendeffort[i][0];
                    foot_state_.effort[i].y = footendeffort[i][1];
                    foot_state_.effort[i].z = footendeffort[i][2];
                }
                if (i < contact.size())
                {
                    foot_state_.contact[i] = contact[i];
                }
            }
        }
    }

    // Special function for Hexapod201 with Python interface
    void setStepBodyPoseCmd(const pinocchio::SE3 &body_pose)
    {
        if (config_.usePyInterface)
        {
            // Publish to python interface
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = config_.odomParentFrame;
            pose_msg.pose.position.x = body_pose.translation()[0];
            pose_msg.pose.position.y = body_pose.translation()[1];
            pose_msg.pose.position.z = body_pose.translation()[2];
            Eigen::Quaterniond quat(body_pose.rotation());
            pose_msg.pose.orientation.x = quat.x();
            pose_msg.pose.orientation.y = quat.y();
            pose_msg.pose.orientation.z = quat.z();
            pose_msg.pose.orientation.w = quat.w();
            pose_cmd_pub.publish(pose_msg);
        }
        // For non-Python interface, just set the body pose
        setBodyPoseCmd(body_pose);
    }

    /**
     * @brief Set the Step Cmd object
     *
     * @param body_pose Body pose in odom frame
     * @param footendpos Foot end positions in body frame
     * @param contact Foot contact states
     * @note This function is only effective when usePyInterface is true.
     */
    void setStepCmd(const pinocchio::SE3 &body_pose,
                    const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<bool> &contact)
    {
        if (config_.usePyInterface)
        {
            setStepBodyPoseCmd(body_pose);
            setFootCmd(footendpos, std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()),
                       std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero()), contact);
        }
    }

    //// Interface extensions
    std::vector<Eigen::Vector3d>
    getNominalFootholds() const
    {
        return nominal_footholds;
    }
};