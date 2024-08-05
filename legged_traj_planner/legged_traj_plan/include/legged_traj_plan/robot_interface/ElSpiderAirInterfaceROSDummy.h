/**
 * @file ElSpiderAirInterfaceROSDummy.h
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

class ElSpiderAirInterfaceROSDummy : public ElSpiderAirInterface
{
private:
    ros::NodeHandle nh;

    // Rviz
    ros::Publisher joint_state_pub;
    ros::Publisher shadow_joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;

    // feedbacks
    ros::Subscriber footfdb_sub;
    legged_traj_plan::FootState foot_state_fdb_;
    ros::Subscriber bodyfdb_sub;
    pinocchio::SE3 body_pose_fdb_;
    pinocchio::Motion body_vel_fdb_;

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
    ElSpiderAirInterfaceROSDummy(const std::string &urdf, bool sim = false);

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

    // Rviz visualization
    /**
     * @brief Publish base pose in case of fake feedback
     * 
     * @param odom 
     * @param child_frame 
     * @param parent_frame 
     */
    void pub_odom(const pinocchio::SE3 &odom,
                  const std::string &child_frame = "base",
                  const std::string &parent_frame = "odom");
    void pub_joint_state(const std::vector<double> &q);
    void pub_joint_state(const std::vector<Eigen::Vector3d> &q);
    void pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);

    void pub_shadow_joint_state(const std::vector<double> &q);
    void pub_shadow_joint_state(const std::vector<Eigen::Vector3d> &q);
    void pub_shadow_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos);


};