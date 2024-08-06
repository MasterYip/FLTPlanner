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
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h"
#include <geometry_msgs/TransformStamped.h>

bool odom2SE3_Motion(const nav_msgs::Odometry &odom, pinocchio::SE3 &pos, pinocchio::Motion &vel)
{
    pos = pinocchio::SE3(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
                         Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
    vel = pinocchio::Motion(Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z),
                            Eigen::Vector3d(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
    return true;
}

[[deprecated("constructor deprecated")]] ElSpiderAirInterfaceROS::ElSpiderAirInterfaceROS(const std::string &urdf, bool sim)
    : ElSpiderAirInterface(urdf), sim_(sim)
{
    ROS_WARN("ElSpiderAirInterfaceROS(const std::string &urdf, bool sim) constructor is deprecated.");
    // Rviz
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    shadow_joint_state_pub = nh.advertise<sensor_msgs::JointState>("shadow/joint_states", 10);
    // Feedback
    footfdb_sub = nh.subscribe("/hexapod/foot_state_fdb", 1, &ElSpiderAirInterfaceROS::footfdbCallback, this);
    bodyfdb_sub = nh.subscribe("/base_odom", 1, &ElSpiderAirInterfaceROS::bodyfdbCallback, this);
    // Cmd
    footcmd_pub = nh.advertise<legged_traj_plan::FootCmd>("/hexapod/hlc/foot_cmd_track", 1);
    jointcmd_pub = nh.advertise<legged_traj_plan::JointCmd>("/hexapod/hlc/joint_cmd_track", 1);
    feedforward_type = 0;
    if (!sim_) // Hardware
    {
        joint_kp = {0.15, 0.15, 0.15};
        joint_kd = {1.0, 1.0, 1.0};
    }
    else // Gazebo
    {
        joint_kp = {1500, 3000, 3000};
        joint_kd = {5, 7.5, 7.5};
    }
}

ElSpiderAirInterfaceROS::ElSpiderAirInterfaceROS(const ElSpiderAirInterfaceROSConfig &config) : config_(config), ElSpiderAirInterface(config.urdf)
{
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(config.jointStateTopic, 10);

    jointfdb_sub = nh.subscribe(config.jointStateFdbTopic, 1, &ElSpiderAirInterfaceROS::jointfdbCallback, this);
    footfdb_sub = nh.subscribe(config.footStateFdbTopic, 1, &ElSpiderAirInterfaceROS::footfdbCallback, this);
    bodyfdb_sub = nh.subscribe(config.bodyStateFdbTopic, 1, &ElSpiderAirInterfaceROS::bodyfdbCallback, this);
    footcmd_pub = nh.advertise<legged_traj_plan::FootCmd>(config.footCmdTopic, 1);
    jointcmd_pub = nh.advertise<legged_traj_plan::JointCmd>(config.jointCmdTopic, 1);

    feedforward_type = 0;
    if (!config.sim) // Hardware
    {
        joint_kp = config.jointKpHardware;
        joint_kd = config.jointKdHardware;
    }
    else // Gazebo
    {
        joint_kp = config.jointKpSim;
        joint_kd = config.jointKdSim;
    }
}

// feedbacks

void ElSpiderAirInterfaceROS::jointfdbCallback(const sensor_msgs::JointState &msg)
{
    joint_state_fdb_ = msg;
    for (int i=0; i<6; ++i)
    {
        joint_state_fdb_.position[i*3+1] = -joint_state_fdb_.position[i*3+1] + 0.5*M_PI;
        joint_state_fdb_.velocity[i*3+1] *= -1;
        joint_state_fdb_.position[i*3+2] += M_PI;
    }
    pub_joint_state(joint_state_fdb_.position);
}

void ElSpiderAirInterfaceROS::footfdbCallback(const legged_traj_plan::FootState &msg)
{
    foot_state_fdb_ = msg;
}

void ElSpiderAirInterfaceROS::bodyfdbCallback(const nav_msgs::Odometry &msg)
{
    odom2SE3_Motion(msg, body_pose_fdb_, body_vel_fdb_);
}

// Commands

void ElSpiderAirInterfaceROS::pub_footcmd_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
{
    legged_traj_plan::FootCmd footcmd;
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
    footcmd_pub.publish(footcmd);
}

void ElSpiderAirInterfaceROS::pub_footcmd_from_footendcmd(const std::vector<Eigen::Vector3d> &footendpos,
                                                          const std::vector<Eigen::Vector3d> &footendvel,
                                                          const std::vector<Eigen::Vector3d> &footendeffort)
{
    legged_traj_plan::FootCmd footcmd;
    footcmd.header.stamp = ros::Time::now();
    footcmd.feedforward_type = feedforward_type; // Default 0, foot feedforward
    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::Point pt;
        pt.x = footendpos[i][0];
        pt.y = footendpos[i][1];
        pt.z = footendpos[i][2];
        footcmd.foot_position.push_back(pt);
        geometry_msgs::Vector3 vec3;
        footcmd.joint_torque.push_back(vec3); // zero, not used
        vec3.x = footendvel[i][0];
        vec3.y = footendvel[i][1];
        vec3.z = footendvel[i][2];
        footcmd.foot_velocity.push_back(vec3);
        vec3.x = footendeffort[i][0];
        vec3.y = footendeffort[i][1];
        vec3.z = footendeffort[i][2];
        footcmd.foot_effort.push_back(vec3);
        vec3.x = joint_kp[0];
        vec3.y = joint_kp[1];
        vec3.z = joint_kp[2];
        footcmd.joint_kp.push_back(vec3);
        vec3.x = joint_kd[0];
        vec3.y = joint_kd[1];
        vec3.z = joint_kd[2];
        footcmd.joint_kd.push_back(vec3);
    }
    footcmd_pub.publish(footcmd);
}

void ElSpiderAirInterfaceROS::pub_jointcmd_from_jointpos(const std::vector<double> &q)
{
    legged_traj_plan::JointCmd jointcmd;
    jointcmd.header.stamp = ros::Time::now();
    jointcmd.position = q;
    jointcmd.velocity = std::vector<double>(18, 0);
    jointcmd.torque = std::vector<double>(18, 0);

    for (int i = 0; i < 6; ++i)
    {
        jointcmd.kp.emplace_back(joint_kp[0]);
        jointcmd.kd.emplace_back(joint_kd[0]);
        jointcmd.kp.emplace_back(joint_kp[1]);
        jointcmd.kd.emplace_back(joint_kd[1]);
        jointcmd.kp.emplace_back(joint_kp[2]);
        jointcmd.kd.emplace_back(joint_kd[2]);
        jointcmd.position[i * 3 + 1] = -jointcmd.position[i * 3 + 1] + 0.5 * M_PI;
        jointcmd.position[i * 3 + 2] -= M_PI;
    }
    jointcmd_pub.publish(jointcmd);
}

void ElSpiderAirInterfaceROS::pub_jointcmd_from_jointpos(const std::vector<Eigen::Vector3d> &q)
{
    std::vector<double> q_vec;
    for (auto pos : q)
    {
        q_vec.push_back(pos[0]);
        q_vec.push_back(pos[1]);
        q_vec.push_back(pos[2]);
    }
    pub_jointcmd_from_jointpos(q_vec);
}

// Rviz

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

void ElSpiderAirInterfaceROS::pub_joint_state(const std::vector<Eigen::Vector3d> &q)
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

void ElSpiderAirInterfaceROS::pub_shadow_joint_state(const std::vector<double> &q)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = SHADOW_JOINT_STATE_NAME;
    joint_state.position = q;
    shadow_joint_state_pub.publish(joint_state);
}

void ElSpiderAirInterfaceROS::pub_shadow_joint_state(const std::vector<Eigen::Vector3d> &q)
{
    std::vector<double> q_vec;
    for (auto pos : q)
    {
        q_vec.push_back(pos[0]);
        q_vec.push_back(pos[1]);
        q_vec.push_back(pos[2]);
    }
    pub_shadow_joint_state(q_vec);
}

void ElSpiderAirInterfaceROS::pub_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
{
    pub_joint_state(IKFast_foots(footendpos));
}

void ElSpiderAirInterfaceROS::pub_shadow_joint_state_from_footendpos(const std::vector<Eigen::Vector3d> &footendpos)
{
    pub_shadow_joint_state(IKFast_foots(footendpos));
}
