/**
 * @file DummyElSpiderAirInterfaceROS.h
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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
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

struct DummyElSpiderAirConfig
{
    std::string urdf;
    std::string jointStateTopic;
    std::string odomChildFrame;
    std::string odomParentFrame;
    bool enableVis;

    // Init
    std::vector<double> nominalFootPos;      // size 18: (xyz in base frame) * 6
    std::vector<double> nominalFootPosShift; // size 3: (dx dy dz)
    std::vector<double> initBodyPose;        // size 6: (xyzrpy)

    void loadParam(ros::NodeHandle &nh)
    {
        bool check_digit = true;
        check_digit &= nh.getParam("/robot_description", urdf);
        check_digit &= nh.getParam("robotInterface/jointStateTopic", jointStateTopic);
        check_digit &= nh.getParam("robotInterface/odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam("robotInterface/odomParentFrame", odomParentFrame);
        check_digit &= nh.getParam("robotInterface/enableVis", enableVis);
        check_digit &= nh.getParam("robotInterface/nominalFootPos", nominalFootPos);
        check_digit &= nominalFootPos.size() == 18;
        check_digit &= nh.getParam("robotInterface/nominalFootPosShift", nominalFootPosShift);
        check_digit &= nominalFootPosShift.size() == 3;
        check_digit &= nh.getParam("robotInterface/initBodyPose", initBodyPose);
        check_digit &= initBodyPose.size() == 6;
        if (!check_digit)
        {
            ROS_ERROR("Failed to load parameters");
        }
    }
};

class DummyElSpiderAirInterfaceROS : public ElSpiderAirInterface
{
private:
    ros::NodeHandle nh;
    DummyElSpiderAirConfig config_;

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
        joint_state.name = JOINT_STATE_NAME;
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
        pub_joint_state(IKFast_foots(footendpos));
    }

public:
    DummyElSpiderAirInterfaceROS(const DummyElSpiderAirConfig &config_) : ElSpiderAirInterface(config_.urdf)
    {
        // Rviz
        joint_state_pub = nh.advertise<sensor_msgs::JointState>(config_.jointStateTopic, 10);
        // Init State
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
            Eigen::Vector3d q_i = IKFast_foot(Eigen::Vector3d(pt.x, pt.y, pt.z), i);
            joint_state_.position.emplace_back(q_i[0]);
            joint_state_.position.emplace_back(q_i[1]);
            joint_state_.position.emplace_back(q_i[2]);
        }
        foot_state_.velocity.resize(6);
        foot_state_.effort.resize(6);

        joint_state_.velocity.resize(18);
        joint_state_.effort.resize(18);
    }

    // Overrides
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
    // FIXME: setJointCmd & setFootCmd should share the same state variable
    void setJointCmd(const std::vector<double> &q) override
    {
        joint_state_.position = q;
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
        joint_state_.position = q_vec;
        if (config_.enableVis)
            pub_joint_state(q_vec);
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        for (int i = 0; i < 6; ++i)
        {
            geometry_msgs::Point pt;
            pt.x = footendpos[i][0];
            pt.y = footendpos[i][1];
            pt.z = footendpos[i][2];
            foot_state_.position[i] = pt;
            // TODO: update joint_state_
        }
        if (config_.enableVis)
            pub_joint_state_from_footendpos(footendpos);
    }
};