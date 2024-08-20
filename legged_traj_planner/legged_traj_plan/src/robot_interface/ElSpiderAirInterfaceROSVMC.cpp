/**
 * @file ElSpiderAirInterfaceROSVMC.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROSVMC.h"

ElSpiderAirInterfaceROSVMC::ElSpiderAirInterfaceROSVMC(const ElSpiderAirInterfaceROSVMCConfig &config)
    : config_(config), ElSpiderAirInterfaceROS(config)
{
    exp_foot_state_pub_ = nh.advertise<legged_traj_plan::FootState>(config.expFootStateTopicName, 1);
    exp_body_state_pub_ = nh.advertise<nav_msgs::Odometry>(config.expPoseTopicName, 1);

    exp_body_state_.header.frame_id = config_.odomParentFrame;
    exp_body_state_.child_frame_id = config_.odomChildFrame;
    exp_foot_state_.position.resize(6);
    exp_foot_state_.velocity.resize(6);
    exp_foot_state_.effort.resize(6);
    exp_foot_state_.contact.resize(6);
}

virtual void ElSpiderAirInterfaceROSVMC::setBodyPoseCmd(const pinocchio::SE3 &body_pose)
{
    exp_body_state_.header.stamp = ros::Time::now();
    exp_body_state_.pose.pose.position.x = body_pose.translation()[0];
    exp_body_state_.pose.pose.position.y = body_pose.translation()[1];
    exp_body_state_.pose.pose.position.z = body_pose.translation()[2];
    Eigen::Quaterniond quat(body_pose.rotation());
    exp_body_state_.pose.pose.orientation.x = quat.x();
    exp_body_state_.pose.pose.orientation.y = quat.y();
    exp_body_state_.pose.pose.orientation.z = quat.z();
    exp_body_state_.pose.pose.orientation.w = quat.w();
    exp_body_state_pub_.publish(exp_body_state_);
}

virtual void ElSpiderAirInterfaceROSVMC::setBodyVelCmd(const pinocchio::Motion &body_vel)
{
    exp_body_state_.header.stamp = ros::Time::now();
    exp_body_state_.twist.twist.linear.x = body_vel.linear()[0];
    exp_body_state_.twist.twist.linear.y = body_vel.linear()[1];
    exp_body_state_.twist.twist.linear.z = body_vel.linear()[2];
    exp_body_state_.twist.twist.angular.x = body_vel.angular()[0];
    exp_body_state_.twist.twist.angular.y = body_vel.angular()[1];
    exp_body_state_.twist.twist.angular.z = body_vel.angular()[2];
    exp_body_state_pub_.publish(exp_body_state_);
}

virtual void ElSpiderAirInterfaceROSVMC::setFootCmd(const std::vector<Eigen::Vector3d> &footendpos)
{
    exp_foot_state_.header.stamp = ros::Time::now();
    exp_foot_state_.position.clear();
    for (const auto &pos : footendpos)
    {
        geometry_msgs::Point point;
        point.x = pos[0];
        point.y = pos[1];
        point.z = pos[2];
        exp_foot_state_.position.emplace_back(point);
    }
    exp_foot_state_pub_.publish(exp_foot_state_);
}
