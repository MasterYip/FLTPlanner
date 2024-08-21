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
#include <vector>
/* external project header files */
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>
#include <hexapod_controller/FootCmd.h>
#include <hexapod_controller/FootState.h>
#include <hexapod_controller/JointCmd.h>
#include <hexapod_controller/JointState.h>

#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include "ros_visualizer/ros_visualizer.hpp"
/* internal project header files */
#include "hexapod_controller/Task.h"
#include "hexapod_controller/MultiDimPID.h"

using hex_contact_flag_t = std::array<bool, 6>;

struct VMCConfig
{
    bool sim; // Gazebo | Hardware
    bool use_joint_cmd;

    double mu;
    double mass;
    std::vector<double> inertia_diag = {3, 1};
    Eigen::Matrix3d inertia;
    double gravity;
    double loop_rate; // Control rate

    // PID gains for expected pose accerleration
    // zeta = Kd/2/sqrt(Kp) for double integrator
    double Kp, Kd;

    double pidgrf_Kp, pidgrf_lim, pidgrf_T, pidgrf_tau;

    // Topics
    std::string fdb_pose_topic_name;
    std::string fdb_pose_topic_name_sim;
    std::string fdb_foot_state_topic_name;

    std::string exp_pose_topic_name;
    std::string exp_foot_state_topic_name;
    std::string footcmd_topic_name;
    std::string exp_joint_state_topic_name;
    std::string jointcmd_topic_name;

    // Frame name for TF looking up
    std::string body_frame;
    std::string world_frame;

    std::vector<double> joint_kp_st = {3, 0}; // FIXME: this init method is wrone
    std::vector<double> joint_kd_st = {3, 0};
    std::vector<double> joint_kp_sw = {3, 0};
    std::vector<double> joint_kd_sw = {3, 0};
    std::vector<double> joint_kp_sim_st = {3, 0};
    std::vector<double> joint_kd_sim_st = {3, 0};
    std::vector<double> joint_kp_sim_sw = {3, 0};
    std::vector<double> joint_kd_sim_sw = {3, 0};

    inline void
    loadParameters(const ros::NodeHandle &nh)
    {
        nh.param("sim", sim, true);
        nh.param("use_joint_cmd", use_joint_cmd, false);
        nh.param("mu", mu, 0.5);
        nh.param("mass", mass, 30.0); // 30
        // TODO
        inertia = Eigen::Matrix3d::Identity();
        nh.param("inertia_diag", inertia_diag, {3, 1});
        inertia.diagonal() << inertia_diag[0], inertia_diag[1], inertia_diag[2];
        nh.param("gravity", gravity, 9.81);
        nh.param("loop_rate", loop_rate, 200.0);
        // Kp Kd for Accerleration PD
        nh.param("Kp", Kp, 1.0);
        nh.param("Kd", Kd, 2.0);

        // Multi-dim PID for grf filtering
        nh.param("pidgrf_Kp", pidgrf_Kp, 1.0);
        nh.param("pidgrf_lim", pidgrf_lim, 100.0);
        pidgrf_T = 1.0 / loop_rate;
        pidgrf_tau = pidgrf_T / 2.0;

        // Topics
        nh.param("fdb_pose_topic_name", fdb_pose_topic_name, std::string("/base_odom"));
        nh.param("fdb_pose_topic_name_sim", fdb_pose_topic_name_sim, std::string("/torso_odom"));
        nh.param("fdb_foot_state_topic_name", fdb_foot_state_topic_name, std::string("/hexapod/foot_state_fdb"));

        nh.param("exp_pose_topic_name", exp_pose_topic_name, std::string("/exp_odom"));
        nh.param("exp_foot_state_topic_name", exp_foot_state_topic_name, std::string("/exp_foot_state"));
        nh.param("footcmd_topic_name", footcmd_topic_name, std::string("/hexapod/hlc/foot_cmd_track"));
        nh.param("exp_joint_state_topic_name", exp_joint_state_topic_name, std::string("/exp_joint_state"));
        nh.param("jointcmd_topic_name", jointcmd_topic_name, std::string("/hexapod/hlc/joint_cmd_track"));

        nh.param("body_frame", body_frame, std::string("base"));
        nh.param("world_frame", world_frame, std::string("odom"));

        if (sim)
        {
            // joint_kp = {1500, 3000, 3000};
            // joint_kd = {5, 7.5, 7.5};
            nh.param("joint_kp_sim_st", joint_kp_sim_st, {100.0, 100.0, 100.0});
            nh.param("joint_kd_sim_st", joint_kd_sim_st, {3.0, 3.0, 3.0});
            nh.param("joint_kp_sim_sw", joint_kp_sim_sw, {100.0, 100.0, 100.0});
            nh.param("joint_kd_sim_sw", joint_kd_sim_sw, {3.0, 3.0, 3.0});
        }
        else
        {
            // joint_kp = {0.1, 0.15, 0.15};
            // joint_kd = {2, 2, 2};
            nh.param("joint_kp_st", joint_kp_st, {0.04, 0.04, 0.04});
            nh.param("joint_kd_st", joint_kd_st, {1.0, 1.0, 1.0});
            nh.param("joint_kp_sw", joint_kp_sw, {0.04, 0.04, 0.04});
            nh.param("joint_kd_sw", joint_kd_sw, {1.0, 1.0, 1.0});
        }

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
    hexapod_controller::JointState exp_joint_state_;
    ros::Subscriber exp_joint_state_sub_;
    bool recv_exp_joint_state_ = false;

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
    hexapod_controller::JointCmd joint_cmd_;
    ros::Publisher joint_cmd_pub_;

    // Misc
    std::vector<MultiDimPID> pid_grf_;
    ros_visualizer::ROSVisualizer rosvis_;
    double last_vis_time_ = 0;
    double vis_interval_ = 0.05;

public:
    VMCController(ros::NodeHandle &nh);
    bool fdbPoseLookup();
    void fdbPoseCallback(const nav_msgs::Odometry &msg);
    void expPoseCallback(const nav_msgs::Odometry &msg);
    void fdbFootStateCallback(const hexapod_controller::FootState &msg);
    void expFootStateCallback(const hexapod_controller::FootState &msg);
    void expJointStateCallback(const hexapod_controller::JointState &msg);

    void pubFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                    const std::vector<Eigen::Vector3d> &footendvel,
                    const std::vector<Eigen::Vector3d> &footendeffort,
                    const hex_contact_flag_t &contact_flag = {true, true, true, true, true, true});

    void pubJointCmd(const std::vector<double> &joint_pos,
                     const std::vector<double> &joint_vel,
                     const std::vector<double> &joint_effort,
                     const hex_contact_flag_t &contact_flag = {true, true, true, true, true, true});

    void controllLoop();
    void run();

    // Tests
    void test_getExpAcc();
    void test_getGrf();
    void test_fdbCalcGrf();
};