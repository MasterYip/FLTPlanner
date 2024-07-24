/**
 * @file ElSpiderAirVMCPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterfaceROS.h" // Should be included first (pinocchio)
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_plan/whole_body_planner/WholeBodyPlanner.h"

#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/BodyState.h"
// MCTS
#include "contactPlannerInterface.h"
#include "planning.h"
#include "myDataType.h"
#include "HexapodParameter.h"
#include "user.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "legged_traj_search/utils/gcs_visualizer.hpp"

// BUG
// IMPORTANT: Add this function to avoid Convex hull display error. (unknown reason)
void AVOID_DISPLAY_ERROR(void)
{
    Eigen::Vector3d vec(1, 1, 1);
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vec.data(), vec.cols(), false, false);
}

legged_traj_plan::hexapod_State transRobotState(const MDT::RobotState &state_)
{
    legged_traj_plan::hexapod_State hexapodState;
    hexapodState.base_Pose_Now.position.x = state_.pose.x;
    hexapodState.base_Pose_Now.position.y = state_.pose.y;
    hexapodState.base_Pose_Now.position.z = state_.pose.z;
    hexapodState.base_Pose_Now.orientation.roll = state_.pose.roll;
    hexapodState.base_Pose_Now.orientation.pitch = state_.pose.pitch;
    hexapodState.base_Pose_Now.orientation.yaw = state_.pose.yaw;
    hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;

    for (int i = 0; i < 6; i++)
    {
        hexapodState.feetPositionNow.foot[i].x = state_.feetPosition[i].x();
        hexapodState.feetPositionNow.foot[i].y = state_.feetPosition[i].y();
        hexapodState.feetPositionNow.foot[i].z = state_.feetPosition[i].z();
        hexapodState.support_State_Now[i] = !state_.gaitToNow[i];
        hexapodState.faultLeg_State_Now[i] = MDT::NORMAL_LEG_FLAG;
    }
    hexapodState.move_Direction.x = cos(state_.moveDirection);
    hexapodState.move_Direction.y = sin(state_.moveDirection);
    hexapodState.move_Direction.z = 0;
    // 下一步落足点
    hexapodState.feetPositionNext = hexapodState.feetPositionNow;
    // 下一步支撑状态和容错状态
    hexapodState.support_State_Next = hexapodState.support_State_Now;
    hexapodState.faultLeg_State_Next = hexapodState.faultLeg_State_Now;
    return hexapodState;
}

// 初始化机器人状态,并赋初值
MDT::RobotState initRobotState(const MDT::Pose &robotPoseW, MDT::Vector6b gaitToNow, float moveDirection)
{
    MDT::RobotState state_;
    state_.initialize();
    state_.pose = robotPoseW;
    state_.faultStateToNow << MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG, MDT::NORMAL_LEG_FLAG;
    for (int i = 0; i < 6; i++)
    {
        // PLANNING::POINT pnt = {HexapodParameter::transList[i].x, HexapodParameter::transList[i].y, HexapodParameter::transList[i].z};
        state_.feetPosition[i] = MDT::pointRotationAndTrans(HexapodParameter::norminalFoothold_B[i], robotPoseW.getT_W_B());
        state_.feetNormalVector[i] << 0, 0, 1; // 默认法向量竖直向上
        state_.gaitToNow[i] = gaitToNow[i];
        state_.maxNormalForce[i] = 1000.0f;
        state_.frcitionMu[i] = 0.8f;
    }
    state_.moveDirection = moveDirection;

    return state_;
}

void randomizeRobotState(MDT::RobotState &state_, double noise_amp = 0.1)
{
    for (int i = 0; i < 6; i++)
    {
        state_.feetPosition[i] += Eigen::Vector3d((rand() % 200 - 100) / 100.0 * noise_amp,
                                                  (rand() % 200 - 100) / 100.0 * noise_amp,
                                                  (rand() % 200 - 100) / 100.0 * noise_amp);
    }
    state_.pose.x += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.y += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.z += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.roll += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.pitch += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.pose.yaw += (rand() % 200 - 100) / 100.0 * noise_amp;
    state_.moveDirection += (rand() % 200 - 100) / 100.0 * noise_amp;
}

MDT::RobotState getInitState(MDT::Pose robotPose = {1, 0, USER::norminalTrunkHeight, 0, 0, 0 * _PI_ / 6},
                             float moveDir = 0)
{
    MDT::Vector6b gaitToNow;
    gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
    return initRobotState(robotPose, gaitToNow, moveDir);
}

class ElSpiderAirVMCPlanner // Force compensation planner
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;

    // Joystick cmd subscribe
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    /// Feedback subscribe
    // Foot state feedback
    ros::Subscriber foot_state_sub_;
    legged_traj_plan::FootState foot_state_;
    bool recv_foot_state_ = false;

    /// Command publish
    ros::Publisher exp_foot_state_pub_;
    legged_traj_plan::FootState exp_foot_state_;
    ros::Publisher exp_body_state_pub_;
    nav_msgs::Odometry exp_body_state_;

    // Odometry
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped body_state_tf_;
    bool recv_body_state_ = false;

    /// Interface
    // Fast legged planner interface
    std::shared_ptr<ElSpiderAirInterfaceROS> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    MCTSWholeBodyPlanner whole_body_planner_;

    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<Eigen::Vector3f> exp_path_;
    float multiply_factor_ = 0.03;
    int point_num_ = 3;

    /// Misc
    // ROS Timer event
    ros::Timer timer_;

    // Visualizer
    GCSVisualizer visualizer_;

    // Settings
    bool fake_estimation_;
    bool fake_estimation_noisy_ = false;
    double noise_amp_ = 0.02;
    bool simulation_;

public:
    // FIXME: use ros param to init gridmap_interface_
    ElSpiderAirVMCPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                          bool fake_estimation = false, bool simulation = false) : nh_("~"),
                                                                                   robot_interface_(std::make_shared<ElSpiderAirInterfaceROS>(nh_.param("/robot_description", std::string("")), simulation)),
                                                                                   gridmap_interface_(std::make_shared<GridMapInterface>(nh_, "/grid_map")),
                                                                                   whole_body_planner_(swing_traj_planner_config, gridmap_interface_, robot_interface_),
                                                                                   tfListener_(tfBuffer_), visualizer_(nh_, "odom", "vmc_planner_marker"),
                                                                                   rate_(20), fake_estimation_(fake_estimation), simulation_(simulation)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirVMCPlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 1, &ElSpiderAirVMCPlanner::foot_state_callback, this);
        exp_foot_state_pub_ = nh_.advertise<legged_traj_plan::FootState>("/exp_foot_state", 1);
        exp_body_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/exp_odom", 1);
        exp_foot_state_.position.resize(6);
        exp_foot_state_.velocity.resize(6);
        exp_foot_state_.effort.resize(6);
        exp_foot_state_.contact.resize(6);

        robot_state_.initialize();
        next_planned_state_.initialize();
        if (fake_estimation_)
        {
            robot_state_ = getInitState();
            next_planned_state_ = robot_state_;
        }
        else
        {
            timer_ = nh_.createTimer(ros::Duration(0.05), &ElSpiderAirVMCPlanner::timer_callback, this);
        }
    }

    //// Callbacks
    void timer_callback(const ros::TimerEvent &event)
    {
        pub_jointstate();
        // Update body state
        try
        {
            // Use ros::Time(0) to prevent warning of `extrapolate to future`
            body_state_tf_ = tfBuffer_.lookupTransform("odom", "base", ros::Time(0));
            recv_body_state_ = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

    // Joystick cmd callback
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // Start planning
        if ((recv_foot_state_ && recv_body_state_) || fake_estimation_)
        {
            update_exp_path();
            update_robot_state();
            gridmap_interface_->lockMapUpdate();
            // 1.MCTS
            bool ret = CONTACT_PLANNER::pathTrackPlanner(robot_state_, next_planned_state_, exp_path_,
                                                         gridmap_interface_->getMap(), true, 400);
            // 2. Triple gait
            // bool ret = true;
            // next_planned_state_ = CONTACT_PLANNER::tripleGaitPlanner(robot_state_, gridmap_interface_->getMap(), 0.1);

            // SwingTraj Vis Clear
            whole_body_planner_.visClear();

            // Vis getAvailableFootholds
            // MDT::AvailableContactsInfo available_points = PLANNING::getAvailableFootholds_visual(next_planned_state_, gridmap_interface_->getMap());
            // std::vector<Eigen::Vector3d> pts;
            // for (int i = 0; i < 6; i++)
            // {
            //     for (auto pt : available_points.position.leg[i])
            //     {
            //         pts.emplace_back(pt);
            //     }
            // }
            // visualizer_.delAll();
            // visualizer_.visSphere(pts, 0.01);

            // Vis next foothold
            visualizer_.delAll();
            auto hexapod_state = transRobotState(next_planned_state_);
            for (int i = 0; i < 6; i++)
            {
                Eigen::Vector3d pt = {hexapod_state.feetPositionNow.foot[i].x, hexapod_state.feetPositionNow.foot[i].y, hexapod_state.feetPositionNow.foot[i].z};
                visualizer_.visSphere(pt, 0.02);
            }

            gridmap_interface_->unlockMapUpdate();
            if (ret)
            {
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(next_planned_state_));
            }
            else
            {
                ROS_INFO("MCTS failed to plan, reset to nominal state.");
                whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_),
                                                        transRobotState(getInitState(robot_state_.pose, robot_state_.moveDirection)));
            }
            traj_planner();
        }
    }

    // Foot state feedback callback
    void foot_state_callback(const legged_traj_plan::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    //// Rviz
    // Pub Real Robot JointState for Rviz
    void pub_jointstate(void)
    {
        std::vector<Eigen::Vector3d> footend_now;
        if (recv_foot_state_ && recv_body_state_)
        {
            update_robot_state();
        }
        for (size_t k = 0; k < 6; ++k)
        {
            tf2::Transform transform;
            tf2::Quaternion quaternion;
            quaternion.setRPY(robot_state_.pose.roll, robot_state_.pose.pitch, robot_state_.pose.yaw);
            transform.setOrigin(tf2::Vector3(robot_state_.pose.x, robot_state_.pose.y, robot_state_.pose.z));
            transform.setRotation(quaternion);
            Eigen::Isometry3d pose = tf2::transformToEigen(tf2::toMsg(transform));
            // inverse transform
            footend_now.emplace_back(pose.inverse() * robot_state_.feetPosition[k]);
        }
        robot_interface_->pub_joint_state_from_footendpos(footend_now);
    }

    void state_traj_replay(MCTStateTransfer &state_traj)
    {
        for (double t = 0.0; t < 1.01; t += 0.05)
        {
            // Get Interpolated State
            auto odom_interp = state_traj.eval_torso_traj(t);
            auto footend_interp = state_traj.eval_foot_traj(t); // Footend position in world frame
            auto support_state = state_traj.eval_support_state(t);
            for (size_t k = 0; k < 6; ++k)
            {
                // Convert to BASE
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }

            // Visualization
            robot_interface_->pub_odom(odom_interp, "shadowbase", "odom");
            robot_interface_->pub_shadow_joint_state_from_footendpos(footend_interp);
            ros::spinOnce();  // Fetch feedback
            pub_jointstate(); // Publish real joint state

            rate_.sleep();
        }
    }

    //// MCTS Interface
    // Update Robot State for MCTS Interface
    void update_robot_state(void)
    {
        if (fake_estimation_)
        {
            robot_state_ = next_planned_state_;
            if (fake_estimation_noisy_)
            {
                randomizeRobotState(robot_state_, noise_amp_);
            }
        }
        else
        {
            // TODO: time stamp?
            robot_state_.pose.x = body_state_tf_.transform.translation.x;
            robot_state_.pose.y = body_state_tf_.transform.translation.y;
            robot_state_.pose.z = body_state_tf_.transform.translation.z;
            // RPY
            tf2::Quaternion q;
            tf2::fromMsg(body_state_tf_.transform.rotation, q);
            tf2::Matrix3x3(q).getRPY(robot_state_.pose.roll, robot_state_.pose.pitch, robot_state_.pose.yaw);

            // FIXME: gaitToNow? default 0
            std::vector<Eigen::Vector3d> footend_vis;
            for (int i = 0; i < 6; ++i)
            {
                robot_state_.gaitToNow[i] = MDT::SUPPORT_FLAG; // use FootState.contact?
                robot_state_.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                // Absolute foot position
                robot_state_.feetPosition[i] = tf2::transformToEigen(body_state_tf_.transform) *
                                               Eigen::Vector3d(foot_state_.position[i].x, foot_state_.position[i].y, foot_state_.position[i].z);
                robot_state_.feetNormalVector[i] << 0, 0, 1; // TODO: use gridmap normal
                footend_vis.emplace_back(robot_state_.feetPosition[i]);
            }

            // FIXME: cmd_ should be under robot frame
            // if (cmd_.linear.x != 0)
            //     robot_state_.moveDirection = atan2(cmd_.linear.y, cmd_.linear.x);
            robot_state_.moveDirection = robot_state_.pose.yaw;
            // TODO: maxNormalForce, frictionMu
        }
    }

    // Update Exp Path for MCTS Interface
    void update_exp_path(void)
    {
        exp_path_.clear();
        // FIXME: pose.z should be on torso height map!!! (Not used temporarily in MCTS)
        double height = 0.25;
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x, robot_state_.pose.y, height));
        for (int i = 0; i < point_num_; ++i)
        {
            exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + (cmd_.linear.x * cos(robot_state_.pose.yaw) - cmd_.linear.y * sin(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                robot_state_.pose.y + (cmd_.linear.x * sin(robot_state_.pose.yaw) + cmd_.linear.y * cos(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                height));
        }
    }

    //// VMC Interface
    void pub_exp_pose(pinocchio::SE3 exp_pose, geometry_msgs::Twist cmd)
    {
        exp_body_state_.header.stamp = ros::Time::now();
        exp_body_state_.header.frame_id = "odom";
        exp_body_state_.pose.pose.position.x = exp_pose.translation()(0);
        exp_body_state_.pose.pose.position.y = exp_pose.translation()(1);
        exp_body_state_.pose.pose.position.z = exp_pose.translation()(2);
        Eigen::Quaterniond quat(exp_pose.rotation());
        exp_body_state_.pose.pose.orientation.x = quat.x();
        exp_body_state_.pose.pose.orientation.y = quat.y();
        exp_body_state_.pose.pose.orientation.z = quat.z();
        exp_body_state_.pose.pose.orientation.w = quat.w();
        geometry_msgs::Twist twist_world;
        Eigen::Vector3d linear_world;
        linear_world << cmd.linear.x, cmd.linear.y, cmd.linear.z;
        linear_world = exp_pose.rotation() * linear_world;
        Eigen::Vector3d angular_world;
        angular_world << cmd.angular.x, cmd.angular.y, cmd.angular.z;
        angular_world = exp_pose.rotation() * angular_world;
        twist_world.linear.x = linear_world(0);
        twist_world.linear.y = linear_world(1);
        twist_world.linear.z = linear_world(2);
        twist_world.angular.x = angular_world(0);
        twist_world.angular.y = angular_world(1);
        twist_world.angular.z = angular_world(2);
        exp_body_state_.twist.twist = twist_world;
        exp_body_state_pub_.publish(exp_body_state_);
    }

    void pub_exp_footstate(std::vector<Eigen::Vector3d> footend_interp, std::array<bool, 6> support_state, bool check_contact = false)
    {
        exp_foot_state_.header.stamp = ros::Time::now();
        for (size_t k = 0; k < 6; ++k)
        {
            if (!(check_contact && is_contact(k, 0.2) && !support_state[k])) // for swing leg, if already in contact, then skip
            {
                geometry_msgs::Point pt;
                pt.x = footend_interp[k][0];
                pt.y = footend_interp[k][1];
                pt.z = footend_interp[k][2];
                exp_foot_state_.position[k] = pt;
                exp_foot_state_.contact[k] = support_state[k];
            }
        }
        exp_foot_state_pub_.publish(exp_foot_state_);
    }

    void pub_vmc_exp_state(MCTStateTransfer &state_traj, double t)
    {
        // Get Interpolated State
        auto odom_interp = state_traj.eval_torso_traj(t);
        auto footend_interp = state_traj.eval_foot_traj(t); // Footend position in world frame
        auto support_state = state_traj.eval_support_state(t);
        for (size_t k = 0; k < 6; ++k)
        {
            // Convert to BASE
            footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
        }
        // VMC exp state
        pub_exp_footstate(footend_interp, support_state, t > 0.5);
        // pub_exp_footstate(footend_interp, support_state, false);
        pub_exp_pose(odom_interp, geometry_msgs::Twist());
    }

    //// Contact Handling (Sim only)
    // FIXME: avoid error detection
    bool is_contact(int leg_idx, double eps = 0.1)
    {
        Eigen::Vector3d foot_force;
        foot_force << foot_state_.effort[leg_idx].x, foot_state_.effort[leg_idx].y, foot_state_.effort[leg_idx].z;
        return foot_force.norm() > eps;
    }

    void stance_contact_handle(void)
    {
        bool flag = false;
        int max_cnt = 50;
        // double adj_height = 0.005;
        double alpha = 0.01;
        ROS_INFO("Stance contact handling...");
        while (!flag && max_cnt-- > 0)
        {
            flag = true;
            for (size_t k = 0; k < 6; ++k)
            {
                if (is_contact(k) == false)
                {
                    flag = false;
                    // exp_foot_state_.position[k].z -= adj_height;
                    exp_foot_state_.position[k].x = LOWEST_FOOT_POS[k](0) * alpha + exp_foot_state_.position[k].x * (1 - alpha);
                    exp_foot_state_.position[k].y = LOWEST_FOOT_POS[k](1) * alpha + exp_foot_state_.position[k].y * (1 - alpha);
                    exp_foot_state_.position[k].z = LOWEST_FOOT_POS[k](2) * alpha + exp_foot_state_.position[k].z * (1 - alpha);
                }
            }
            exp_foot_state_pub_.publish(exp_foot_state_);
            ros::spinOnce(); // Fetch feedback
            rate_.sleep();
        }
        if (max_cnt <= 0)
            ROS_WARN("Stance contact handling failed.");
        else
            ROS_INFO("Stance contact handling done.");
    }

    //// Planning
    // for lift & touch smoothing
    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }

    void traj_planner()
    {
        double t = 0.0;
        double delta = 0.02;
        MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
        state_traj_replay(state_traj);
        std::cout << "Press space to execute trajectory...";
        getchar();
        do
        {
            ros::spinOnce(); // Fetch feedback
            pub_vmc_exp_state(state_traj, sine_remap(t));

            // Visualization
            pub_jointstate(); // Publish real joint state

            t += delta;
            rate_.sleep();
            if (t > 1.0)
            {
                // Publish last state point (commented out due to foot contact detection)
                // pub_vmc_exp_state(state_traj, 1);
                t = 0.0;
                whole_body_planner_.dequeue_MCTsolution();
                if (whole_body_planner_.get_state_traj_length() > 0)
                    state_traj = whole_body_planner_.get_state_traj(0);
            }
        } while (whole_body_planner_.get_state_traj_length() > 0);

        // Set all foot contact to true
        for (size_t k = 0; k < 6; ++k)
            exp_foot_state_.contact[k] = true;
        exp_foot_state_pub_.publish(exp_foot_state_);

        ros::spinOnce();  // Fetch feedback
        pub_jointstate(); // Publish real joint state
        ros::Duration(0.4).sleep();
        // Stance contact handling
        stance_contact_handle();
    }

    void run()
    {
        ros::spin();
    }
};
