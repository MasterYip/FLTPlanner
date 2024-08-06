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

    void loadParam(ros::NodeHandle &nh)
    {
        bool check_digit = true;
        check_digit &= nh.getParam("urdf", urdf);
        check_digit &= nh.getParam("jointStateTopic", jointStateTopic);
        check_digit &= nh.getParam("odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam("odomParentFrame", odomParentFrame);
        check_digit &= nh.getParam("enableVis", enableVis);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load parameters");
        }
    }
}

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
    DummyElSpiderAirInterfaceROS(const DummyElSpiderAirConfig &config) : ElSpiderAirInterface(config.urdf)
    {
        // Rviz
        joint_state_pub = nh.advertise<sensor_msgs::JointState>(config.jointStateTopic, 10);
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
        if (config.enableVis)
            pub_odom(body_pose);
    }
    // FIXME: setJointCmd & setFootCmd should share the same state variable
    void setJointCmd(const std::vector<double> &q) override
    {
        joint_state_.position = q;
        if (config.enableVis)
            pub_joint_state(q);
    }

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        for (int i = 0; i < 6; ++i)
        {
            geometry_msgs::Point pt;
            pt.x = footendpos[i][0];
            pt.y = footendpos[i][1];
            pt.z = footendpos[i][2];
            foot_state_.foot_pos[i] = pt;
            // TODO: update joint_state_
        }
        if (config.enableVis)
            pub_joint_state_from_footendpos(footendpos);
    }
};