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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/JointCmd.h"

/* internal project header files */

class ElSpiderAirInterfaceROS : public ElSpiderAirInterface
{
private:
    ros::NodeHandle nh;
    // Rviz
    ros::Publisher joint_state_pub;
    ros::Publisher shadow_joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;
    // Commands
    // Foot command
    ros::Publisher footcmd_pub;
    int feedforward_type;
    // Joint command
    ros::Publisher jointcmd_pub;
    std::vector<double> joint_kp;
    std::vector<double> joint_kd;
    // Misc
    bool sim_;

public:
    ElSpiderAirInterfaceROS(const std::string &urdf, bool sim = false);

    bool setJointKpKd(const std::vector<double> &kp, const std::vector<double> &kd)
    {
        if (kp.size() == 3 && kd.size() == 3)
        {
            joint_kp = kp;
            joint_kd = kd;
            return true;
        }
        ROS_WARN("Invalid kp or kd size");
        return false;
    }

    // Rviz
    void pub_odom(const pinocchio::SE3 &odom,
                  const std::string &child_frame = "base",
                  const std::string &parent_frame = "odom");
    void pub_joint_state(const std::vector<double> &q);
    void pub_joint_state(const std::vector<Eigen::Vector3d> &q);
    void pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);

    void pub_shadow_joint_state(const std::vector<double> &q);
    void pub_shadow_joint_state(const std::vector<Eigen::Vector3d> &q);
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

    void pub_jointcmd_from_jointpos(const std::vector<double> &q);

    void pub_jointcmd_from_jointpos(const std::vector<Eigen::Vector3d> &q);
};