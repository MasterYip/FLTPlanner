/**
 * @file vmc_controller.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "hexapod_controller/vmc_controller.hpp"

pinocchio::SE3 transformToSE3(const geometry_msgs::TransformStamped &tf)
{
    return pinocchio::SE3(Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                                             tf.transform.rotation.y, tf.transform.rotation.z),
                          Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
}

VMCController::VMCController(ros::NodeHandle &nh) : tfListener_(tfBuffer_)
{
    cfg_.loadParameters(nh);
    exp_foot_state_sub_ = nh.subscribe(cfg_.exp_foot_state_topic_name, 1, &VMCController::expFootStateCallback, this);
    fdb_foot_state_sub_ = nh.subscribe(cfg_.fdb_foot_state_topic_name, 1, &VMCController::fdbFootStateCallback, this);
    exp_pose_sub_ = nh.subscribe(cfg_.exp_pose_topic_name, 1, &VMCController::expPoseCallback, this);
    foot_cmd_pub_ = nh.advertise<hexapod_controller::FootCmd>(cfg_.footcmd_topic_name, 1);
}

bool VMCController::fdbPoseLookup()
{
    try
    {
        // FIXME: Make sure it is updated in real-time
        body_state_tf_ = tfBuffer_.lookupTransform(cfg_.world_frame, cfg_.body_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not get body state: %s", ex.what());
        return false;
    }
    fdb_pose_ = transformToSE3(body_state_tf_);
    recv_fdb_pose_ = true;
    return true;
}

void VMCController::expPoseCallback(const geometry_msgs::TransformStamped &msg)
{
    exp_pose_ = transformToSE3(msg);
    recv_exp_pose_ = true;
}

void VMCController::fdbFootStateCallback(const hexapod_controller::FootState &msg)
{
    fdb_foot_state_ = msg;
    recv_fdb_foot_state_ = true;
}

void VMCController::expFootStateCallback(const hexapod_controller::FootState &msg)
{
    exp_foot_state_ = msg;
    recv_exp_foot_state_ = true;
}

void VMCController::controllLoop()
{
    fdbPoseLookup();
    if (recv_exp_pose_ && recv_exp_foot_state_ && recv_fdb_foot_state_ && recv_fdb_pose_)
    {
        // TODO
    }
    else
    {
        ROS_WARN("Not all states are received");
    }
}

void VMCController::run()
{
    ros::Rate loop_rate(cfg_.loop_rate);
    while (ros::ok())
    {
        // TODO
        ros::spinOnce();
        loop_rate.sleep();
    }
}