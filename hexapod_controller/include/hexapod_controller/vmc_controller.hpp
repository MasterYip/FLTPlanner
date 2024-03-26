/**
 * @file vmc_controller.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>
#include <hexapod_controller/FootCmd.h>
#include <hexapod_controller/FootState.h>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include "ros_visualizer/ros_visualizer.hpp"
/* internal project header files */
#include "hexapod_controller/Task.h"

using hex_contact_flag_t = std::array<bool, 6>;

struct VMCConfig
{
    double mu;
    double mass;
    Eigen::Matrix3d inertia;
    double gravity;
    double loop_rate; // Control rate

    // PID gains for expected pose accerleration
    // zeta = Kd/2/sqrt(Kp) for double integrator
    double Kp, Kd;

    // Topics
    std::string exp_pose_topic_name;
    std::string fdb_pose_topic_name;
    std::string exp_foot_state_topic_name;
    std::string fdb_foot_state_topic_name;
    std::string footcmd_topic_name;
    // Frame name for TF looking up
    std::string body_frame;
    std::string world_frame;

    inline void
    loadParameters(const ros::NodeHandle &nh)
    {
        nh.param("mu", mu, 0.5);
        nh.param("mass", mass, 30.0);
        // TODO
        inertia = Eigen::Matrix3d::Identity();
        inertia.diagonal() << 0.3, 0.4, 0.5;
        nh.param("gravity", gravity, 9.81);
        nh.param("loop_rate", loop_rate, 200.0);

        nh.param("Kp", Kp, 1.0);
        nh.param("Kd", Kd, 2.0);
        // Topics
        nh.param("exp_pose_topic_name", exp_pose_topic_name, std::string("/exp_odom"));
        nh.param("fdb_pose_topic_name", fdb_pose_topic_name, std::string("/torso_odom"));
        nh.param("exp_foot_state_topic_name", exp_foot_state_topic_name, std::string("/exp_foot_state"));
        nh.param("fdb_foot_state_topic_name", fdb_foot_state_topic_name, std::string("/hexapod/foot_state_fdb"));
        nh.param("footcmd_topic_name", footcmd_topic_name, std::string("/hexapod/hlc/foot_cmd_track"));

        nh.param("body_frame", body_frame, std::string("base"));
        nh.param("world_frame", world_frame, std::string("odom"));
        return;
    };
};

class VMCController
{

private:
    VMCConfig cfg_;

    // State Expected
    // pose
    pinocchio::SE3 exp_pose_;
    pinocchio::Motion exp_vel_;
    ros::Subscriber exp_pose_sub_;
    bool recv_exp_pose_ = false;

    // foot_pos
    hexapod_controller::FootState exp_foot_state_;
    ros::Subscriber exp_foot_state_sub_;
    bool recv_exp_foot_state_ = false;

    // State Feedback
    // pose
    // Use tf (not used temporarily)
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped body_state_tf_;
    // Use gazebo odom
    pinocchio::SE3 fdb_pose_;
    pinocchio::Motion fdb_vel_;
    ros::Subscriber fdb_pose_sub_;
    bool recv_fdb_pose_ = false;

    // foot_pos
    hexapod_controller::FootState fdb_foot_state_;
    ros::Subscriber fdb_foot_state_sub_;
    bool recv_fdb_foot_state_ = false;

    // Control Commands
    hexapod_controller::FootCmd foot_cmd_;
    ros::Publisher foot_cmd_pub_;

    // Misc
    ros_visualizer::ROSVisualizer rosvis_;

public:
    VMCController(ros::NodeHandle &nh);
    bool fdbPoseLookup();
    void fdbPoseCallback(const nav_msgs::Odometry &msg);
    void expPoseCallback(const nav_msgs::Odometry &msg);
    void fdbFootStateCallback(const hexapod_controller::FootState &msg);
    void expFootStateCallback(const hexapod_controller::FootState &msg);

    void controllLoop();
    void run();

    // Tests
    void test_getExpAcc();
    void test_getGrf();
    void test_fdbCalcGrf();
};