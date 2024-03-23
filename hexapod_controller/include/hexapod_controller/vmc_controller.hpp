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
#include <pinocchio/spatial/log.hpp>
#include <hexapod_controller/FootCmd.h>
#include <hexapod_controller/FootState.h>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
/* internal project header files */

struct VMCConfig
{
    double mu;
    double mass;
    double gravity;
    double loop_rate;

    std::string exp_pose_topic_name;
    std::string exp_foot_state_topic_name;
    std::string fdb_foot_state_topic_name;
    std::string footcmd_topic_name;
    std::string body_frame;
    std::string world_frame;

    inline void
    loadParameters(const ros::NodeHandle &nh)
    {
        nh.param("mu", mu, 0.5);
        nh.param("mass", mass, 30.0);
        nh.param("gravity", gravity, 9.81);
        nh.param("loop_rate", loop_rate, 200.0);
        nh.param("exp_pose_topic_name", exp_pose_topic_name, std::string("/exp_pose"));
        nh.param("exp_foot_state_topic_name", exp_foot_state_topic_name, std::string("/exp_foot_state"));
        nh.param("fdb_foot_state_topic_name", fdb_foot_state_topic_name, std::string("/fdb_foot_state"));
        nh.param("footcmd_topic_name", footcmd_topic_name, std::string("/footcmd_track"));
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
    ros::Subscriber exp_pose_sub_;
    bool recv_exp_pose_ = false;
    // foot_pos
    hexapod_controller::FootState exp_foot_state_;
    ros::Subscriber exp_foot_state_sub_;
    bool recv_exp_foot_state_ = false;

    // State Feedback
    // pose
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped body_state_tf_;
    pinocchio::SE3 fdb_pose_; // Feedback
    bool recv_fdb_pose_ = false;
    // foot_pos
    hexapod_controller::FootState fdb_foot_state_;
    ros::Subscriber fdb_foot_state_sub_;
    bool recv_fdb_foot_state_ = false;

    // Control Commands
    hexapod_controller::FootCmd foot_cmd_;
    ros::Publisher foot_cmd_pub_;

public:
    VMCController(ros::NodeHandle &nh);
    bool fdbPoseLookup();
    void expPoseCallback(const geometry_msgs::TransformStamped &msg);
    void fdbFootStateCallback(const hexapod_controller::FootState &msg);
    void expFootStateCallback(const hexapod_controller::FootState &msg);
    bool getExpWrench(const pinocchio::SE3 &com_pos,
                      const pinocchio::SE3 &exp_pos,
                      pinocchio::Force &exp_wrench);
    bool getGroundReactionForce(const pinocchio::Force &exp_wrench,
                                const std::vector<Eigen::Vector3d> foot_pos,
                                std::vector<Eigen::Vector3d> &grf);
    void controllLoop();
    void run();
};