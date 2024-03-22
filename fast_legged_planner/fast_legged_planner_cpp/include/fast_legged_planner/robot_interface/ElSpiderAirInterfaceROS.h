/**
 * @file ElSpiderAirInterfaceROS.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "fast_legged_planner/robot_interface/ElSpiderAirInterface.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include "fast_legged_planner/FootCmd.h"

/* internal project header files */

class ElSpiderAirInterfaceROS : public ElSpiderAirInterface
{
private:
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub;
    ros::Publisher shadow_joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;
    ros::Publisher foot_pos_pub;
    int feedforward_type;
    std::vector<double> joint_kp;
    std::vector<double> joint_kd;
    bool sim_;

public:
    ElSpiderAirInterfaceROS(const std::string &urdf, bool sim = false);

    // Rviz
    void pub_odom(const pinocchio::SE3 &odom,
                  const std::string &child_frame = "base",
                  const std::string &parent_frame = "odom");
    void pub_joint_state(const std::vector<double> &q);
    void pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);
    void pub_shadow_joint_state(const std::vector<double> &q);
    void pub_shadow_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);

    // HexapodSoftware
    /**
     * @brief Publish foot command from foot end position
     * @note Interface with HexapodSoftware HLC
     * @param footendpos
     */
    void pub_footcmd_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);

    void pub_footcmd_from_footendcmd(const std::vector<Eigen::Vector3d> &footendpos,
                                     const std::vector<Eigen::Vector3d> &footendvel,
                                     const std::vector<Eigen::Vector3d> &footendeffort);
};