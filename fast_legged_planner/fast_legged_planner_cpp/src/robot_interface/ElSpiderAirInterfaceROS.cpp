#include "fast_legged_planner/robot_interface/ElSpiderAirInterfaceROS.h"

ElSpiderAirInterfaceROS::ElSpiderAirInterfaceROS(const std::string &urdf)
    : ElSpiderAirInterface(urdf)
{
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    foot_pos_pub = nh.advertise<fast_legged_planner::FootCmd>("/hexapod/hlc/foot_cmd_track", 1);
    feedforward_type = 0;
    joint_kp = {0.075, 0.2, 0.2};
    joint_kd = {2, 2, 2};
}

void ElSpiderAirInterfaceROS::pub_footcmd_from_footendpos(const std::vector<std::vector<double>> &footendpos)
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
        vec3.x = joint_kp[0];
        vec3.y = joint_kp[1];
        vec3.z = joint_kp[2];
        footcmd.joint_kp.push_back(vec3);
        vec3.x = joint_kd[0];
        vec3.y = joint_kd[1];
        vec3.z = joint_kd[2];
        footcmd.joint_kd.push_back(vec3);
        footcmd.joint_torque.push_back(vec3);
    }
    foot_pos_pub.publish(footcmd);
}

void ElSpiderAirInterfaceROS::pub_odom(const pinocchio::SE3 &odom,
                                            const std::string &child_frame,
                                            const std::string &parent_frame)
{
    tf::Vector3 xyz(odom.translation().x(), odom.translation().y(), odom.translation().z());
    tf::Quaternion quat(odom.rotation().x(), odom.rotation().y(), odom.rotation().z(), odom.rotation().w());
    odom_pub.sendTransform(tf::StampedTransform(tf::Transform(quat, xyz), ros::Time::now(), child_frame, parent_frame));
}

void ElSpiderAirInterfaceROS::pub_joint_state(const std::vector<double> &q)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = JOINT_STATE_NAME;
    joint_state.position = q;
    joint_state_pub.publish(joint_state);
}

void ElSpiderAirInterfaceROS::pub_joint_state_from_footendpos(const std::vector<std::vector<double>> &footendpos)
{
    std::vector<double> q = IKFast_foots(footendpos);
    pub_joint_state(q);
}
