/**
 * @file ElSpiderAirInterfaceROS.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "fast_legged_planner/robot_interface/ElSpiderAirInterfaceROS.h"
#include <geometry_msgs/TransformStamped.h>

ElSpiderAirInterfaceROS::ElSpiderAirInterfaceROS(const std::string &urdf, bool sim)
    : ElSpiderAirInterface(urdf), sim_(sim)
{
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    foot_pos_pub = nh.advertise<fast_legged_planner::FootCmd>("/hexapod/hlc/foot_cmd_track", 1);
    feedforward_type = 0;
    if (!sim_) // Hardware
    {
        joint_kp = {0.1, 0.15, 0.15};
        joint_kd = {2, 2, 2};
    }
    else // Gazebo
    {
        joint_kp = {1000, 1500, 1500};
        joint_kd = {5, 7.5, 7.5};
    }
}

void ElSpiderAirInterfaceROS::pub_footcmd_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
{
    fast_legged_planner::FootCmd footcmd;
    footcmd.header.stamp = ros::Time::now();
    footcmd.feedforward_type = feedforward_type;
    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::Point pt;
        pt.x = footendpos[i][0];
        pt.y = footendpos[i][1];
        pt.z = footendpos[i][2];
        footcmd.foot_position.push_back(pt);
        geometry_msgs::Vector3 vec3;
        footcmd.foot_velocity.push_back(vec3);
        footcmd.foot_effort.push_back(vec3);
        footcmd.joint_torque.push_back(vec3);
        vec3.x = joint_kp[0];
        vec3.y = joint_kp[1];
        vec3.z = joint_kp[2];
        footcmd.joint_kp.push_back(vec3);
        vec3.x = joint_kd[0];
        vec3.y = joint_kd[1];
        vec3.z = joint_kd[2];
        footcmd.joint_kd.push_back(vec3);
    }
    foot_pos_pub.publish(footcmd);
}

void ElSpiderAirInterfaceROS::pub_odom(const pinocchio::SE3 &odom,
                                       const std::string &child_frame,
                                       const std::string &parent_frame)
{
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = parent_frame;
    odom_tf.child_frame_id = child_frame;
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

void ElSpiderAirInterfaceROS::pub_joint_state(const std::vector<double> &q)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = JOINT_STATE_NAME;
    joint_state.position = q;
    joint_state_pub.publish(joint_state);
}

void ElSpiderAirInterfaceROS::pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
{
    pub_joint_state(IKFast_foots(footendpos));
}
