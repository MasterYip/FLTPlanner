/**
 * @file test_vmc_stance.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/math/rpy.hpp>

#include <vector>

#include <ros/ros.h>
#include "legged_traj_plan/FootState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct TestVMCCmdPubCfg
{
    std::vector<int> contact_state;

    void loadConfig(ros::NodeHandle &nh)
    {
        nh.param("contact_state", contact_state, {1, 1, 1, 1, 1, 1});
    };
};

class TestVMCCmdPub
{
private:
    /// Command publish
    ros::NodeHandle &nh_;

    // Cmd Pub
    ros::Publisher exp_foot_state_pub_;
    legged_traj_plan::FootState exp_foot_state_;
    ros::Publisher exp_body_state_pub_;
    nav_msgs::Odometry exp_body_state_;

    // Use gazebo odom
    pinocchio::SE3 fdb_pose_;
    pinocchio::Motion fdb_vel_;
    ros::Subscriber fdb_pose_sub_;
    bool recv_fdb_pose_ = false;

    // Exp state
    std::vector<Eigen::Vector3d> footpos_;
    pinocchio::SE3 exp_pose_;
    pinocchio::SE3 init_pose_;

    ros::Subscriber joy_sub_;
    sensor_msgs::Joy joy_cmd_;
    Eigen::Vector3d rpy_;

    // Config
    TestVMCCmdPubCfg cfg_;

public:
    TestVMCCmdPub(ros::NodeHandle &nh) : nh_(nh)
    {
        cfg_.loadConfig(nh_);
        exp_foot_state_pub_ = nh_.advertise<legged_traj_plan::FootState>("/exp_foot_state", 1);
        exp_body_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/exp_odom", 1);
        fdb_pose_sub_ = nh_.subscribe("/torso_odom", 1, &TestVMCCmdPub::fdbPoseCallback, this);
        joy_sub_ = nh_.subscribe("/joy", 1, &TestVMCCmdPub::joyCallback, this);

        exp_foot_state_.position.resize(6);
        exp_foot_state_.velocity.resize(6);
        exp_foot_state_.effort.resize(6);
        exp_foot_state_.contact.resize(6);

        footpos_.emplace_back(Eigen::Vector3d(0.354, -0.28 - 0.04, -0.28));
        footpos_.emplace_back(Eigen::Vector3d(0.054, -0.34 - 0.04, -0.28));
        footpos_.emplace_back(Eigen::Vector3d(-0.354, -0.28 - 0.04, -0.28));
        footpos_.emplace_back(Eigen::Vector3d(0.354, 0.28 + 0.04, -0.28));
        footpos_.emplace_back(Eigen::Vector3d(0.054, 0.34 + 0.04, -0.28));
        footpos_.emplace_back(Eigen::Vector3d(-0.354, 0.28 + 0.04, -0.28));
        for (uint k = 0; k < 6; ++k)
        {
            geometry_msgs::Point pt;
            pt.x = footpos_[k][0];
            pt.y = footpos_[k][1];
            pt.z = footpos_[k][2];
            exp_foot_state_.position[k] = pt;
            exp_foot_state_.contact[k] = cfg_.contact_state[k] == 1;
        }

        exp_pose_ = pinocchio::SE3(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0.27));
    }

    bool odom2SE3_Motion(const nav_msgs::Odometry &odom, pinocchio::SE3 &pos, pinocchio::Motion &vel)
    {
        pos = pinocchio::SE3(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
                             Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
        vel = pinocchio::Motion(Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z),
                                Eigen::Vector3d(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
        return true;
    }

    void fdbPoseCallback(const nav_msgs::Odometry &msg)
    {
        if (!recv_fdb_pose_)
        {
            recv_fdb_pose_ = odom2SE3_Motion(msg, fdb_pose_, fdb_vel_);
            init_pose_ = fdb_pose_;
            exp_pose_ = init_pose_;
        }
        else
        {
            odom2SE3_Motion(msg, fdb_pose_, fdb_vel_);
        }
    }

    void joyCallback(const sensor_msgs::Joy &msg)
    {
        joy_cmd_ = msg;
        rpy_[0] = joy_cmd_.axes[0];
        rpy_[1] = joy_cmd_.axes[1];
        rpy_[2] = joy_cmd_.axes[3];
        //rpy to rotation matrix
        Eigen::Matrix3d rotation_matrix = pinocchio::rpy::rpyToMatrix(rpy_);
        // Apply rotation to the initial pose
        exp_pose_.rotation() = rotation_matrix * init_pose_.rotation();
    }

    void loop(void)
    {
        if (recv_fdb_pose_)
        {
            // Expected foot state
            exp_foot_state_.header.stamp = ros::Time::now();
            exp_foot_state_pub_.publish(exp_foot_state_);

            // Exoected body state
            exp_body_state_.header.stamp = ros::Time::now();
            exp_body_state_.pose.pose.position.x = exp_pose_.translation()[0];
            exp_body_state_.pose.pose.position.y = exp_pose_.translation()[1];
            exp_body_state_.pose.pose.position.z = exp_pose_.translation()[2];
            Eigen::Quaterniond quat(exp_pose_.rotation());
            exp_body_state_.pose.pose.orientation.x = quat.x();
            exp_body_state_.pose.pose.orientation.y = quat.y();
            exp_body_state_.pose.pose.orientation.z = quat.z();
            exp_body_state_.pose.pose.orientation.w = quat.w();
            // TODO: Temporarily Set velocity to zero
            exp_body_state_.twist.twist.linear.x = 0.0;
            exp_body_state_.twist.twist.linear.y = 0.0;
            exp_body_state_.twist.twist.linear.z = 0.0;
            exp_body_state_.twist.twist.angular.x = 0.0;
            exp_body_state_.twist.twist.angular.y = 0.0;
            exp_body_state_.twist.twist.angular.z = 0.0;
            exp_body_state_pub_.publish(exp_body_state_);
        }
        else
        {
            ROS_WARN("No fdb pose received!");
            ros::Duration(1.0).sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_vmc_stance");
    ros::NodeHandle nh("~");

    TestVMCCmdPub test_vmc_cmd_pub(nh);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        test_vmc_cmd_pub.loop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}