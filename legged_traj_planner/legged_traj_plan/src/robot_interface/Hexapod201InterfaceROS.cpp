/**
 * @file Hexapod201InterfaceROS.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief Implementation of Hexapod201InterfaceROS
 * @version 0.1
 * @date 2024-12-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/robot_interface/Hexapod201InterfaceROS.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Hexapod201InterfaceROS::Hexapod201InterfaceROS(const Hexapod201InterfaceROSConfig &config)
    : BaseRobotInterface(config.urdf), config_(config), nh_("~"),
      pose_received_(false), foot_state_received_(false)
{
    // Initialize publishers (commands to Python interface)
    pose_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.poseCommandTopic, 10);
    foot_cmd_pub_ = nh_.advertise<legged_traj_plan::FootState>(config_.footCommandTopic, 10);

    // Initialize subscribers (feedback from Python interface)
    current_pose_sub_ = nh_.subscribe(config_.currentPoseTopic, 10, 
                                      &Hexapod201InterfaceROS::currentPoseCallback, this);
    foot_state_sub_ = nh_.subscribe(config_.footStateTopic, 10,
                                    &Hexapod201InterfaceROS::footStateCallback, this);

    // Initialize nominal footholds
    nominal_footholds_.clear();
    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector3d foothold(config_.nominalFootPos[3 * i],
                                config_.nominalFootPos[3 * i + 1],
                                config_.nominalFootPos[3 * i + 2]);
        nominal_footholds_.push_back(foothold);
    }

    // Initialize body pose from config
    Eigen::Vector3d init_pos(config_.initBodyPose[0], config_.initBodyPose[1], config_.initBodyPose[2]);
    Eigen::Vector3d init_rpy(config_.initBodyPose[3], config_.initBodyPose[4], config_.initBodyPose[5]);
    body_pose_ = pinocchio::SE3(pinocchio::rpy::rpyToMatrix(init_rpy), init_pos);

    // Initialize joint state
    joint_state_.name = JOINT_STATE_NAME_;
    joint_state_.position.resize(18, 0.0);
    joint_state_.velocity.resize(18, 0.0);
    joint_state_.effort.resize(18, 0.0);

    // Initialize foot state
    foot_state_.name.clear();
    foot_state_.position.clear();
    foot_state_.velocity.clear();
    foot_state_.effort.clear();
    foot_state_.contact.clear();

    for (int i = 0; i < 6; ++i)
    {
        foot_state_.name.push_back("foot_" + std::to_string(i));
        
        geometry_msgs::Point pt;
        pt.x = nominal_footholds_[i][0];
        pt.y = nominal_footholds_[i][1];
        pt.z = nominal_footholds_[i][2];
        foot_state_.position.push_back(pt);

        geometry_msgs::Vector3 vec3;
        vec3.x = vec3.y = vec3.z = 0.0;
        foot_state_.velocity.push_back(vec3);
        foot_state_.effort.push_back(vec3);

        foot_state_.contact.push_back(true);
    }

    ROS_INFO("Hexapod201InterfaceROS initialized");
    ROS_INFO("Waiting for connection with Python interface...");
}

void Hexapod201InterfaceROS::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Update body pose from feedback
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond quaternion(msg->pose.orientation.w, msg->pose.orientation.x, 
                                 msg->pose.orientation.y, msg->pose.orientation.z);
    
    body_pose_ = pinocchio::SE3(quaternion.toRotationMatrix(), position);
    
    pose_received_ = true;
    last_pose_time_ = ros::Time::now();
}

void Hexapod201InterfaceROS::footStateCallback(const legged_traj_plan::FootState::ConstPtr& msg)
{
    // Update foot state from feedback
    foot_state_ = *msg;
    
    foot_state_received_ = true;
    last_foot_state_time_ = ros::Time::now();
}

//// Kinematics Interface (Dummy implementations - not used for real robot control)
std::vector<double> Hexapod201InterfaceROS::IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos)
{
    return std::vector<double>(18, 0.0);
}

Eigen::Vector3d Hexapod201InterfaceROS::IKFast_foot(const Eigen::Vector3d &footendpos, int index)
{
    return Eigen::Vector3d::Zero();
}

bool Hexapod201InterfaceROS::IKFast_foot(const Eigen::Vector3d &footendpos, Eigen::Vector3d &q_result, int index)
{
    q_result = Eigen::Vector3d::Zero();
    return true;
}

Eigen::Vector3d Hexapod201InterfaceROS::FK_foot(const Eigen::Vector3d &q, int index)
{
    if (index >= 0 && index < nominal_footholds_.size())
        return nominal_footholds_[index];
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d Hexapod201InterfaceROS::FK_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Matrix3Xd Hexapod201InterfaceROS::getJacobian(const Eigen::Vector3d &q, int index)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3Xd Hexapod201InterfaceROS::getJacobianTimeVariation(const Eigen::Vector3d &q, const Eigen::Vector3d &vel, int index)
{
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3Xd Hexapod201InterfaceROS::getJacobian_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx)
{
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d Hexapod201InterfaceROS::getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q, const Eigen::Vector3d &vel,
                                                                          int legIdx, int jointIdx)
{
    return Eigen::Matrix3d::Zero();
}

Eigen::Vector3d Hexapod201InterfaceROS::getNominalFoothold(int index)
{
    if (index >= 0 && index < nominal_footholds_.size())
        return nominal_footholds_[index];
    return Eigen::Vector3d::Zero();
}

//// Feedback Interface
const legged_traj_plan::FootState &Hexapod201InterfaceROS::getFootStateFdb() const
{
    return foot_state_;
}

const sensor_msgs::JointState &Hexapod201InterfaceROS::getJointStateFdb() const
{
    return joint_state_;
}

const pinocchio::SE3 &Hexapod201InterfaceROS::getBodyPoseFdb() const
{
    return body_pose_;
}

const pinocchio::Motion &Hexapod201InterfaceROS::getBodyVelFdb() const
{
    return body_vel_;
}

//// Command Interface (Send commands to Python interface)
void Hexapod201InterfaceROS::setBodyPoseCmd(const pinocchio::SE3 &body_pose)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = config_.odomParentFrame;
    
    pose_msg.pose.position.x = body_pose.translation()[0];
    pose_msg.pose.position.y = body_pose.translation()[1];
    pose_msg.pose.position.z = body_pose.translation()[2];
    
    Eigen::Quaterniond quat(body_pose.rotation());
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    
    pose_cmd_pub_.publish(pose_msg);
    
    ROS_DEBUG("Published pose command: pos=(%f,%f,%f), quat=(%f,%f,%f,%f)",
              pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
              quat.x(), quat.y(), quat.z(), quat.w());
}

void Hexapod201InterfaceROS::setBodyVelCmd(const pinocchio::Motion &body_vel)
{
    // Store velocity for local use - not directly sent to Python interface
    body_vel_ = body_vel;
    
    // Note: If velocity commands are needed, we could add a separate velocity command topic
    ROS_DEBUG("Body velocity command received (not forwarded to Python interface)");
}

void Hexapod201InterfaceROS::setJointCmd(const std::vector<double> &q)
{
    if (q.size() >= 18)
    {
        joint_state_.position = q;
    }
    ROS_DEBUG("Joint command received (converted to foot positions)");
    
    // Convert joint positions to foot positions using dummy FK
    std::vector<Eigen::Vector3d> foot_positions;
    for (int i = 0; i < 6; ++i)
    {
        Eigen::Vector3d joint_pos(q[3*i], q[3*i+1], q[3*i+2]);
        foot_positions.push_back(FK_foot(joint_pos, i));
    }
    
    setFootCmd(foot_positions);
}

void Hexapod201InterfaceROS::setJointCmd(const std::vector<Eigen::Vector3d> &q)
{
    std::vector<double> q_flat;
    for (const auto &joint : q)
    {
        q_flat.push_back(joint[0]);
        q_flat.push_back(joint[1]);
        q_flat.push_back(joint[2]);
    }
    setJointCmd(q_flat);
}

void Hexapod201InterfaceROS::setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact)
{
    setJointCmd(q);
    // Contact information could be used in foot command
}

void Hexapod201InterfaceROS::setJointCmd(const std::vector<Eigen::Vector3d> &q,
                                        const std::vector<Eigen::Vector3d> &v,
                                        const std::vector<Eigen::Vector3d> &tau,
                                        const std::vector<bool> &contact)
{
    setJointCmd(q, contact);
    // Velocity and torque information stored but not directly used
}

void Hexapod201InterfaceROS::setFootCmd(const std::vector<Eigen::Vector3d> &footendpos)
{
    std::vector<Eigen::Vector3d> zero_vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> zero_effort(6, Eigen::Vector3d::Zero());
    std::vector<bool> default_contact(6, true);
    
    setFootCmd(footendpos, zero_vel, zero_effort, default_contact);
}

void Hexapod201InterfaceROS::setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                                       const std::vector<Eigen::Vector3d> &footendvel,
                                       const std::vector<Eigen::Vector3d> &footendeffort,
                                       const std::vector<bool> &contact)
{
    legged_traj_plan::FootState footstate_msg;
    footstate_msg.header.stamp = ros::Time::now();
    footstate_msg.header.frame_id = config_.odomChildFrame;

    // Ensure we have 6 foot names
    for (int i = 0; i < 6; ++i)
    {
        footstate_msg.name.push_back("foot_" + std::to_string(i));
    }

    for (int i = 0; i < 6 && i < footendpos.size(); ++i)
    {
        // Position
        geometry_msgs::Point pt;
        pt.x = footendpos[i][0];
        pt.y = footendpos[i][1];
        pt.z = footendpos[i][2];
        footstate_msg.position.push_back(pt);

        // Velocity
        geometry_msgs::Vector3 vel;
        vel.x = (i < footendvel.size()) ? footendvel[i][0] : 0.0;
        vel.y = (i < footendvel.size()) ? footendvel[i][1] : 0.0;
        vel.z = (i < footendvel.size()) ? footendvel[i][2] : 0.0;
        footstate_msg.velocity.push_back(vel);

        // Effort
        geometry_msgs::Vector3 effort;
        effort.x = (i < footendeffort.size()) ? footendeffort[i][0] : 0.0;
        effort.y = (i < footendeffort.size()) ? footendeffort[i][1] : 0.0;
        effort.z = (i < footendeffort.size()) ? footendeffort[i][2] : 0.0;
        footstate_msg.effort.push_back(effort);

        // Contact state
        bool contact_state = (i < contact.size()) ? contact[i] : true;
        footstate_msg.contact.push_back(contact_state);
    }

    foot_cmd_pub_.publish(footstate_msg);
    
    ROS_DEBUG("Published foot command for %zu feet", footendpos.size());
}

void Hexapod201InterfaceROS::setStepCmd(const pinocchio::SE3 &body_pose,
                                       const std::vector<Eigen::Vector3d> &footendpos,
                                       const std::vector<bool> &contact)
{
    // Send both pose and foot commands
    setBodyPoseCmd(body_pose);
    
    std::vector<Eigen::Vector3d> zero_vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> zero_effort(6, Eigen::Vector3d::Zero());
    setFootCmd(footendpos, zero_vel, zero_effort, contact);
    
    ROS_DEBUG("Published coordinated step command (pose + foot positions)");
}

bool Hexapod201InterfaceROS::isConnected() const
{
    ros::Time current_time = ros::Time::now();
    double timeout = getConnectionTimeout();
    
    bool pose_valid = pose_received_ && 
                      (current_time - last_pose_time_).toSec() < timeout;
    bool foot_state_valid = foot_state_received_ && 
                           (current_time - last_foot_state_time_).toSec() < timeout;
    
    return pose_valid && foot_state_valid;
}

std::vector<Eigen::Vector3d> Hexapod201InterfaceROS::getNominalFootholds() const
{
    return nominal_footholds_;
}