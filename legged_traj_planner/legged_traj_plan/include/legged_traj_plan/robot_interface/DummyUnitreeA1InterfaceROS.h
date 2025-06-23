/**
 * @file DummyUnitreeA1InterfaceROS.h
 * @author GitHub Copilot
 * @brief Dummy Unitree A1 Robot Interface with ROS integration
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

/* external project header files */
#include <pinocchio/math/rpy.hpp>
#include "legged_traj_plan/robot_interface/UnitreeA1Interface.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/JointCmd.h"
#include "legged_traj_plan/FootState.h"

/* internal project header files */

struct DummyUnitreeA1InterfaceROSConfig
{
    std::string urdfParamPath;
    std::string urdf; // Auto loaded

    std::string jointStateTopic;
    std::string jointNamePrefix;
    std::string odomChildFrame;
    std::string odomParentFrame;
    bool enableVis;

    // Init
    std::vector<double> nominalFootPos;      // size 12: (xyz in base frame) * 4
    std::vector<double> nominalFootPosShift; // size 3: (dx dy dz)
    std::vector<double> initBodyPose;        // size 6: (xyzrpy)

    void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/urdfParamPath", urdfParamPath);
        check_digit &= nh.getParam(urdfParamPath, urdf);
        check_digit &= nh.getParam(ns + "/jointStateTopic", jointStateTopic);
        check_digit &= nh.getParam(ns + "/jointNamePrefix", jointNamePrefix);
        check_digit &= nh.getParam(ns + "/odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam(ns + "/odomParentFrame", odomParentFrame);
        check_digit &= nh.getParam(ns + "/enableVis", enableVis);
        check_digit &= nh.getParam(ns + "/nominalFootPos", nominalFootPos);
        check_digit &= nominalFootPos.size() == 12; // 4 legs * 3 coordinates
        check_digit &= nh.getParam(ns + "/nominalFootPosShift", nominalFootPosShift);
        check_digit &= nominalFootPosShift.size() == 3;
        check_digit &= nh.getParam(ns + "/initBodyPose", initBodyPose);
        check_digit &= initBodyPose.size() == 6;
        if (!check_digit)
        {
            ROS_ERROR("Failed to load DummyUnitreeA1InterfaceROSConfig");
        }
    }
};

class DummyUnitreeA1InterfaceROS : public UnitreeA1Interface
{
private:
    ros::NodeHandle nh;
    DummyUnitreeA1InterfaceROSConfig config_;

    // Rviz
    ros::Publisher joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;

    // states
    legged_traj_plan::FootState foot_state_;
    sensor_msgs::JointState joint_state_;
    pinocchio::SE3 body_pose_;
    pinocchio::Motion body_vel_;

    // Rviz visualization
    /**
     * @brief Publish base pose in case of fake feedback
     *
     * @param odom
     * @param child_frame
     * @param parent_frame
     */
    void pub_odom(const pinocchio::SE3 &odom)
    {
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

    void pub_joint_state(const std::vector<double> &q)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = A1_JOINT_STATE_NAME;
        if (config_.jointNamePrefix != "")
        {
            for (auto &name : joint_state.name)
                name = config_.jointNamePrefix + name;
        }
        joint_state.position = q;
        joint_state_pub.publish(joint_state);
    }

    void pub_joint_state(const std::vector<Eigen::Vector3d> &q)
    {
        std::vector<double> q_vec;
        for (auto pos : q)
        {
            q_vec.push_back(pos[0]);
            q_vec.push_back(pos[1]);
            q_vec.push_back(pos[2]);
        }
        pub_joint_state(q_vec);
    }

    void pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
    {
        std::vector<double> q_vec;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Vector3d q_i = IK_foot(footendpos[i], i);
            q_vec.push_back(q_i[0]);
            q_vec.push_back(q_i[1]);
            q_vec.push_back(q_i[2]);
        }
        pub_joint_state(q_vec);
    }

public:
    DummyUnitreeA1InterfaceROS(const DummyUnitreeA1InterfaceROSConfig &config) : UnitreeA1Interface(config.urdf), config_(config)
    {
        // Rviz
        joint_state_pub = nh.advertise<sensor_msgs::JointState>(config_.jointStateTopic, 10);
        // Init State
        for (int i = 0; i < 4; i++) // 4 legs for A1
        {
            geometry_msgs::Point pt;
            pt.x = config_.nominalFootPos[3 * i] + config_.nominalFootPosShift[0];
            // A1 leg indexing: FR, FL, RR, RL
            if (i == 0 || i == 2) // FR, RR (right legs)
                pt.y = config_.nominalFootPos[3 * i + 1] - config_.nominalFootPosShift[1];
            else // FL, RL (left legs)
                pt.y = config_.nominalFootPos[3 * i + 1] + config_.nominalFootPosShift[1];
            pt.z = config_.nominalFootPos[3 * i + 2] + config_.nominalFootPosShift[2];
            foot_state_.position.emplace_back(pt);
            Eigen::Vector3d q_i = IK_foot(Eigen::Vector3d(pt.x, pt.y, pt.z), i);
            joint_state_.position.emplace_back(q_i[0]);
            joint_state_.position.emplace_back(q_i[1]);
            joint_state_.position.emplace_back(q_i[2]);
        }
        foot_state_.velocity.resize(4);
        foot_state_.effort.resize(4);
        // FIXME: contact state default to true
        foot_state_.contact = {true, true, true, true};

        joint_state_.velocity.resize(12); // 4 legs * 3 joints
        joint_state_.effort.resize(12);

        body_pose_ = pinocchio::SE3(pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(config_.initBodyPose[3], config_.initBodyPose[4], config_.initBodyPose[5])),
                                    Eigen::Vector3d(config_.initBodyPose[0], config_.initBodyPose[1], config_.initBodyPose[2]));
    }

    //// Overrides
    // Feedback Interface
    const legged_traj_plan::FootState &getFootStateFdb() const override
    {
        return foot_state_;
    }

    const sensor_msgs::JointState &getJointStateFdb() const override
    {
        return joint_state_;
    }

    const pinocchio::SE3 &getBodyPoseFdb() const override
    {
        return body_pose_;
    }

    const pinocchio::Motion &getBodyVelFdb() const override
    {
        return body_vel_;
    }

    // Command Interface
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override
    {
        body_pose_ = body_pose;
        if (config_.enableVis)
            pub_odom(body_pose);
    }

    void setJointCmd(const std::vector<double> &q) override
    {
        joint_state_.position = q;
        // update foot_state_
        for (int i = 0; i < 4; ++i) // 4 legs for A1
        {
            Eigen::Vector3d foot_i = FK_foot(Eigen::Vector3d(q[i * 3], q[i * 3 + 1], q[i * 3 + 2]), i);
            foot_state_.position[i].x = foot_i[0];
            foot_state_.position[i].y = foot_i[1];
            foot_state_.position[i].z = foot_i[2];
        }
        if (config_.enableVis)
            pub_joint_state(q);
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q) override
    {
        std::vector<double> q_vec;
        for (auto pos : q)
        {
            q_vec.emplace_back(pos[0]);
            q_vec.emplace_back(pos[1]);
            q_vec.emplace_back(pos[2]);
        }
        setJointCmd(q_vec);
    }

    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact) override
    {
        setJointCmd(q);
    }

    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                             const std::vector<Eigen::Vector3d> &v,
                             const std::vector<Eigen::Vector3d> &tau,
                             const std::vector<bool> &contact) override
    {
        setJointCmd(q);
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
        {
            geometry_msgs::Point pt;
            pt.x = footendpos[i][0];
            pt.y = footendpos[i][1];
            pt.z = footendpos[i][2];
            foot_state_.position[i] = pt;
            // update joint_state_
            Eigen::Vector3d q_i = IK_foot(footendpos[i], i);
            joint_state_.position[i * 3] = q_i[0];
            joint_state_.position[i * 3 + 1] = q_i[1];
            joint_state_.position[i * 3 + 2] = q_i[2];
        }
        if (config_.enableVis)
            pub_joint_state_from_footendpos(footendpos);
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact) override
    {
        setFootCmd(footendpos);
    }

    // Interface extension
    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> contact)
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
        {
            geometry_msgs::Point pt;
            pt.x = footendpos[i][0];
            pt.y = footendpos[i][1];
            pt.z = footendpos[i][2];
            foot_state_.position[i] = pt;
            foot_state_.velocity[i].x = footendvel[i][0];
            foot_state_.velocity[i].y = footendvel[i][1];
            foot_state_.velocity[i].z = footendvel[i][2];
            foot_state_.effort[i].x = footendeffort[i][0];
            foot_state_.effort[i].y = footendeffort[i][1];
            foot_state_.effort[i].z = footendeffort[i][2];
            foot_state_.contact[i] = contact[i];
            // update joint_state_
            Eigen::Vector3d q_i = IK_foot(footendpos[i], i);
            joint_state_.position[i * 3] = q_i[0];
            joint_state_.position[i * 3 + 1] = q_i[1];
            joint_state_.position[i * 3 + 2] = q_i[2];
        }
        if (config_.enableVis)
            pub_joint_state_from_footendpos(footendpos);
    }
};