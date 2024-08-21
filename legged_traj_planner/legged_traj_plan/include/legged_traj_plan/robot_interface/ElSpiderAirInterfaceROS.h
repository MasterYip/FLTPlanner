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
#include <nav_msgs/Odometry.h>
#include "legged_traj_plan/FootCmd.h"
#include "legged_traj_plan/JointCmd.h"
#include "legged_traj_plan/FootState.h"

/* internal project header files */

struct ElSpiderAirInterfaceROSConfig
{

    std::string urdfParamPath;
    std::string urdf; // Auto loaded

    // Rviz
    bool enableVis;
    std::string jointStateTopic;
    std::string jointNamePrefix;
    std::string odomChildFrame;
    std::string odomParentFrame;

    // Feedback
    std::string jointStateFdbTopic;
    std::string footStateFdbTopic;
    std::string bodyStateFdbTopic;

    // Command
    std::string footCmdTopic;
    std::string jointCmdTopic;

    // Control Params
    bool sim;
    std::vector<double> jointKpSim;
    std::vector<double> jointKdSim;
    std::vector<double> jointKpHardware;
    std::vector<double> jointKdHardware;

    void loadParam(ros::NodeHandle &nh, std::string ns = "robotInterface")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/urdfParamPath", urdfParamPath);
        check_digit &= nh.getParam(urdfParamPath, urdf);

        check_digit &= nh.getParam(ns + "/enableVis", enableVis);
        check_digit &= nh.getParam(ns + "/jointStateTopic", jointStateTopic);
        check_digit &= nh.getParam(ns + "/jointNamePrefix", jointNamePrefix);
        check_digit &= nh.getParam(ns + "/odomChildFrame", odomChildFrame);
        check_digit &= nh.getParam(ns + "/odomParentFrame", odomParentFrame);

        check_digit &= nh.getParam(ns + "/jointStateFdbTopic", jointStateFdbTopic);
        check_digit &= nh.getParam(ns + "/footStateFdbTopic", footStateFdbTopic);
        check_digit &= nh.getParam(ns + "/bodyStateFdbTopic", bodyStateFdbTopic);
        check_digit &= nh.getParam(ns + "/footCmdTopic", footCmdTopic);
        check_digit &= nh.getParam(ns + "/jointCmdTopic", jointCmdTopic);

        check_digit &= nh.getParam(ns + "/sim", sim);
        check_digit &= nh.getParam(ns + "/jointKpSim", jointKpSim);
        check_digit &= jointKpSim.size() == 3;
        check_digit &= nh.getParam(ns + "/jointKdSim", jointKdSim);
        check_digit &= jointKdSim.size() == 3;
        check_digit &= nh.getParam(ns + "/jointKpHardware", jointKpHardware);
        check_digit &= jointKpHardware.size() == 3;
        check_digit &= nh.getParam(ns + "/jointKdHardware", jointKdHardware);
        check_digit &= jointKdHardware.size() == 3;

        if (!check_digit)
        {
            ROS_ERROR("Failed to load ElSpiderAirInterfaceROSConfig");
        }
    }
};

class ElSpiderAirInterfaceROS : public ElSpiderAirInterface
{
protected:
    ros::NodeHandle nh;
    ElSpiderAirInterfaceROSConfig config_; // New added

    // Rviz
    ros::Publisher joint_state_pub;
    ros::Publisher shadow_joint_state_pub;
    tf2_ros::TransformBroadcaster odom_pub;

    // feedbacks
    ros::Subscriber jointfdb_sub;
    sensor_msgs::JointState joint_state_fdb_;
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
    // Reserve for backward compatibility
    ElSpiderAirInterfaceROS(const std::string &urdf, bool sim = false);

    ElSpiderAirInterfaceROS(const ElSpiderAirInterfaceROSConfig &config);

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

    // HexapodSoftware HLC feedbacks
    void jointfdbCallback(const sensor_msgs::JointState &msg);
    void footfdbCallback(const legged_traj_plan::FootState &msg);
    void bodyfdbCallback(const nav_msgs::Odometry &msg);

    // HexapodSoftware HLC commands
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

    // overrides
    // const sensor_msgs::JointState &getJointStateFdb() const { return joint_state_fdb_; }
    virtual const legged_traj_plan::FootState &getFootStateFdb() const override { return foot_state_fdb_; }
    virtual const pinocchio::SE3 &getBodyPoseFdb() const override { return body_pose_fdb_; }
    virtual const pinocchio::Motion &getBodyVelFdb() const override { return body_vel_fdb_; }

    virtual void setBodyPoseCmd(const pinocchio::SE3 &body_pose) override {}; // Not used
    virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) override
    {
        pub_footcmd_from_footendpos(footendpos);
    };

    virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                            const std::vector<Eigen::Vector3d> &footendvel,
                            const std::vector<Eigen::Vector3d> &footendeffort,
                            const std::vector<bool> &contact) override
    {
        setFootCmd(footendpos);
    }

    virtual void setJointCmd(const std::vector<double> &q) override
    {
        pub_jointcmd_from_jointpos(q);
    };

    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q, const std::vector<bool> &contact) override
    {
        pub_jointcmd_from_jointpos(q);
    }

    virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q) override
    {
        pub_jointcmd_from_jointpos(q);
    };
};