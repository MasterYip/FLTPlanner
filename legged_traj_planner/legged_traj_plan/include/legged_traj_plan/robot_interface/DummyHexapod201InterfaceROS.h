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
#include <pinocchio/math/rpy.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/JointCmd.h"
#include "legged_traj_plan/FootState.h"
#include "ros_visualizer/ros_visualizer.hpp"

/* internal project header files */

struct DummyHexapod201InterfaceROSConfig
{
    std::string odomChildFrame;
    std::string odomParentFrame;
    bool enableVis;

    // Init
    std::vector<double> nominalFootPos;      // size 18: (xyz in base frame) * 6
    std::vector<double> nominalFootPosShift; // size 3: (dx dy dz)
    std::vector<double> initBodyPose;        // size 6: (xyzrpy)

    void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam(ns + "/odomParentFrame", odomParentFrame);
        check_digit &= nh.getParam(ns + "/enableVis", enableVis);
        check_digit &= nh.getParam(ns + "/nominalFootPos", nominalFootPos);
        check_digit &= nominalFootPos.size() == 18;
        check_digit &= nh.getParam(ns + "/nominalFootPosShift", nominalFootPosShift);
        check_digit &= nominalFootPosShift.size() == 3;
        check_digit &= nh.getParam(ns + "/initBodyPose", initBodyPose);
        check_digit &= initBodyPose.size() == 6;
        if (!check_digit)
        {
            ROS_ERROR("Failed to load DummyHexapod201InterfaceROSConfig");
        }
    }
};

class DummyHexapod201InterfaceROS
{
private:
    ros::NodeHandle nh;
    DummyHexapod201InterfaceROSConfig config_;

    // Rviz
    tf2_ros::TransformBroadcaster odom_pub;
    std::shared_ptr<ros_visualizer::ROSVisualizer> visualizer_;

    // states
    legged_traj_plan::FootState foot_state_;
    sensor_msgs::JointState joint_state_;
    pinocchio::SE3 body_pose_;
    pinocchio::Motion body_vel_;
    std::vector<Eigen::Vector3d> nominal_footholds;

    // Visualization helpers
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

    void vis_foot_positions(const std::vector<Eigen::Vector3d> &footendpos)
    {
        if (!config_.enableVis || !visualizer_)
            return;
        
        // Visualize feet as spheres
        visualizer_->setIdGroup(0);
        visualizer_->visSphere(footendpos, 0.03, ros_visualizer::VisStyle(1.0, 0.0, 0.0, 1.0, 0.03));
    }

    void vis_body_pose(const pinocchio::SE3 &body_pose)
    {
        if (!config_.enableVis || !visualizer_)
            return;
        
        // Visualize body as cube
        visualizer_->setIdGroup(1);
        Eigen::Vector3d body_pos = body_pose.translation();
        Eigen::Quaterniond quat(body_pose.rotation());
        Eigen::Vector4d quat_vec(quat.w(), quat.x(), quat.y(), quat.z());
        visualizer_->visCube(body_pos, quat_vec, ros_visualizer::VisStyle(0.0, 1.0, 0.0, 0.8, 0.3, 0.2, 0.1));
    }

public:
    DummyHexapod201InterfaceROS(const DummyHexapod201InterfaceROSConfig &config) : config_(config)
    {
        // Initialize visualizer
        if (config_.enableVis)
        {
            visualizer_ = std::make_shared<ros_visualizer::ROSVisualizer>(nh, "odom", "hexapod201_markers");
        }

        // Init State
        nominal_footholds.clear();
        foot_state_.position.clear();
        foot_state_.velocity.clear();
        foot_state_.effort.clear();
        
        for (int i = 0; i < 6; i++)
        {
            geometry_msgs::Point pt;
            pt.x = config_.nominalFootPos[3 * i] + config_.nominalFootPosShift[0];
            if (i < 3)
                pt.y = config_.nominalFootPos[3 * i + 1] - config_.nominalFootPosShift[1];
            else
                pt.y = config_.nominalFootPos[3 * i + 1] + config_.nominalFootPosShift[1];
            pt.z = config_.nominalFootPos[3 * i + 2] + config_.nominalFootPosShift[2];
            foot_state_.position.emplace_back(pt);
            
            // Update nominal foot position
            nominal_footholds.emplace_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }
        
        foot_state_.velocity.resize(6);
        foot_state_.effort.resize(6);
        foot_state_.contact = {true, true, true, true, true, true};

        // Joint state is meaningless for this interface but we initialize it for compatibility
        joint_state_.position.resize(18, 0.0);
        joint_state_.velocity.resize(18, 0.0);
        joint_state_.effort.resize(18, 0.0);

        body_pose_ = pinocchio::SE3(pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(config_.initBodyPose[3], config_.initBodyPose[4], config_.initBodyPose[5])),
                                    Eigen::Vector3d(config_.initBodyPose[0], config_.initBodyPose[1], config_.initBodyPose[2]));
    }

    // Feedback Interface
    const legged_traj_plan::FootState &getFootStateFdb() const
    {
        return foot_state_;
    }

    const sensor_msgs::JointState &getJointStateFdb() const
    {
        return joint_state_;
    }

    const pinocchio::SE3 &getBodyPoseFdb() const
    {
        return body_pose_;
    }

    const pinocchio::Motion &getBodyVelFdb() const
    {
        return body_vel_;
    }

    Eigen::Vector3d getNominalFoothold(int index) const
    {
        if (index >= 0 && index < nominal_footholds.size())
            return nominal_footholds[index];
        return Eigen::Vector3d::Zero();
    }

    std::vector<Eigen::Vector3d> getNominalFootholds() const
    {
        return nominal_footholds;
    }

    // Command Interface
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose)
    {
        body_pose_ = body_pose;
        if (config_.enableVis)
        {
            pub_odom(body_pose);
            vis_body_pose(body_pose);
        }
    }

    void setBodyVelCmd(const pinocchio::Motion &body_vel)
    {
        body_vel_ = body_vel;
    }

    // Dummy joint commands - these don't do anything meaningful since we don't have kinematics
    void setJointCmd(const std::vector<double> &q)
    {
        if (q.size() >= 18)
        {
            joint_state_.position = q;
        }
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q)
    {
        joint_state_.position.clear();
        for (const auto &pos : q)
        {
            joint_state_.position.emplace_back(pos[0]);
            joint_state_.position.emplace_back(pos[1]);
            joint_state_.position.emplace_back(pos[2]);
        }
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact)
    {
        setJointCmd(q);
        if (contact.size() >= 6)
        {
            foot_state_.contact = contact;
        }
    }

    void setJointCmd(const std::vector<Eigen::Vector3d> &q, 
                     const std::vector<Eigen::Vector3d> &v,
                     const std::vector<Eigen::Vector3d> &tau,
                     const std::vector<bool> &contact)
    {
        setJointCmd(q, contact);
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos)
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

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact)
    {
        setFootCmd(footendpos);
        
        for (int i = 0; i < 6 && i < footendvel.size(); ++i)
        {
            foot_state_.velocity[i].x = footendvel[i][0];
            foot_state_.velocity[i].y = footendvel[i][1];
            foot_state_.velocity[i].z = footendvel[i][2];
        }
        
        for (int i = 0; i < 6 && i < footendeffort.size(); ++i)
        {
            foot_state_.effort[i].x = footendeffort[i][0];
            foot_state_.effort[i].y = footendeffort[i][1];
            foot_state_.effort[i].z = footendeffort[i][2];
        }
        
        if (contact.size() >= 6)
        {
            foot_state_.contact = contact;
        }
    }
};