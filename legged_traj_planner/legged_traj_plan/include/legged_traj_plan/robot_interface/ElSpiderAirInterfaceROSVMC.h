/**
 * @file ElSpiderAirInterfaceROSVMC.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/JointCmd.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/JointState.h"

/* internal project header files */

struct ElSpiderAirInterfaceROSVMCConfig : public ElSpiderAirInterfaceROSConfig
{
    std::string expPoseTopicName;
    std::string expFootStateTopicName;
    std::string expJointStateTopicName;

    void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface")
    {
        //  load base class param
        ElSpiderAirInterfaceROSConfig::loadParam(nh, ns);
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/expPoseTopicName", expPoseTopicName);
        check_digit &= nh.getParam(ns + "/expFootStateTopicName", expFootStateTopicName);
        check_digit &= nh.getParam(ns + "/expJointStateTopicName", expJointStateTopicName);
        if (!check_digit)
        {
            throw std::runtime_error("Failed to load ElSpiderAirInterfaceROSVMCConfig");
        }
    }
};

class ElSpiderAirInterfaceROSVMC : public ElSpiderAirInterfaceROS
{
private:
    ElSpiderAirInterfaceROSVMCConfig config_;

    /// Command publish
    ros::Publisher exp_foot_state_pub_;
    legged_traj_plan::FootState exp_foot_state_;
    ros::Publisher exp_joint_state_pub_;
    legged_traj_plan::JointState exp_joint_state_;
    ros::Publisher exp_body_state_pub_;
    nav_msgs::Odometry exp_body_state_;

public:
    ElSpiderAirInterfaceROSVMC(const ElSpiderAirInterfaceROSVMCConfig &config);
    // Command Interface
    void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override;

    void setBodyVelCmd(const pinocchio::Motion &body_vel) override;

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override;

    void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const std::vector<bool> &contact) override;

    void setJointCmd(const std::vector<double> &q) override
    {
        throw std::runtime_error("Not implemented");
    }
    void setJointCmd(const std::vector<Eigen::Vector3d> &q) override;

    void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact) override;
};