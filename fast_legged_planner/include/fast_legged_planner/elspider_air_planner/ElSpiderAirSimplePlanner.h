/**
 * @file ElSpiderAirSimplePlanner.h
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
#include "fast_legged_planner/robot_interface/ElSpiderAirInterfaceROS.h" // Should be included first (pinocchio)
#include "fast_legged_planner/swing_leg_planner/SwingTrajPlanner.h"
#include "fast_legged_planner/perception_interface/GridMapInterface.h"
#include "fast_legged_planner/whole_body_planner/WholeBodyPlanner.h"

#include "fast_legged_planner/hexapod_State.h"
#include "fast_legged_planner/FootState.h"
#include "fast_legged_planner/BodyState.h"
// MCTS
#include "contactPlannerInterface.h"
#include "myDataType.h"
#include "HexapodParameter.h"
#include "user.h"

/* external project header files */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

fast_legged_planner::hexapod_State transRobotState(const MDT::RobotState &state_)
{
    fast_legged_planner::hexapod_State hexapodState;
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

class ElSpiderAirSimplePlanner
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;
    // HexapodSoftware Interface
    ros::Subscriber foot_state_sub_;
    fast_legged_planner::FootState foot_state_;
    bool recv_foot_state_ = false;
    ros::Subscriber body_state_sub_;
    fast_legged_planner::BodyState body_state_;
    bool recv_body_state_ = false;
    // MCTS planner Interface
    MDT::RobotState robot_state_;
    MDT::RobotState next_planned_state_;
    std::vector<Eigen::Vector3f> exp_path_;
    float multiply_factor_ = 0.03;
    int point_num_ = 3;

    // Interface
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    // std::vector<hexapod_State> MCT_solution_;

    // Settings
    bool fake_estimation_;

public:
    ElSpiderAirSimplePlanner(bool fake_estimation = false) : nh_(), robot_interface_(nh_.param("robot_description", std::string(""))),
                                                             gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                                                             rate_(20), fake_estimation_(fake_estimation)
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 100, &ElSpiderAirSimplePlanner::cmd_callback, this);
        foot_state_sub_ = nh_.subscribe("/hexapod/foot_state_fdb", 100, &ElSpiderAirSimplePlanner::foot_state_callback, this);
        body_state_sub_ = nh_.subscribe("/hexapod/body_state_fdb", 100, &ElSpiderAirSimplePlanner::body_state_callback, this);

        robot_state_.initialize();
        next_planned_state_.initialize();
        if (fake_estimation_)
        {
            // 初始化机器人状态,并赋初值
            MDT::Pose robotPoseW = {1, 0, USER::norminalTrunkHeight, 0, 0, 0 * _PI_ / 6};
            MDT::Vector6b gaitToNow;
            gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
            float moveDir = 0 * _PI_ / 2;
            robot_state_ = initRobotState(robotPoseW, gaitToNow, moveDir);
            next_planned_state_ = robot_state_;
        }
    }

    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        // Start planning
        if ((recv_foot_state_ && recv_body_state_) || fake_estimation_)
        {
            update_exp_path();
            // update_exp_path_test();
            update_robot_state();
            next_planned_state_ = CONTACT_PLANNER::pathTrackPlanner(robot_state_, exp_path_, gridmap_interface_.getMap(), true);
            whole_body_planner_.enqueue_MCTsolution(transRobotState(robot_state_), transRobotState(next_planned_state_));
            traj_planner();
        }
    }

    void foot_state_callback(const fast_legged_planner::FootState &msg)
    {
        recv_foot_state_ = true;
        foot_state_ = msg;
    }

    void body_state_callback(const fast_legged_planner::BodyState &msg)
    {
        recv_body_state_ = true;
        body_state_ = msg;
    }

    void update_robot_state(void)
    {
        if (fake_estimation_)
        {
            robot_state_ = next_planned_state_;
        }
        else
        {
            // TODO: time stamp?
            robot_state_.pose.x = body_state_.pose.position.x;
            robot_state_.pose.y = body_state_.pose.position.y;
            robot_state_.pose.z = body_state_.pose.position.z;
            robot_state_.pose.roll = body_state_.eular.roll;
            robot_state_.pose.pitch = body_state_.eular.pitch;
            robot_state_.pose.yaw = body_state_.eular.yaw;
            // FIXME: gaitToNow? default 0
            for (int i = 0; i < 6; ++i)
            {
                robot_state_.gaitToNow[i] = MDT::SUPPORT_FLAG; // use FootState.contact?
                robot_state_.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                robot_state_.feetPosition[i].x() = foot_state_.position[i].x;
                robot_state_.feetPosition[i].y() = foot_state_.position[i].y;
                robot_state_.feetPosition[i].z() = foot_state_.position[i].z;
                robot_state_.feetNormalVector[i] << 0, 0, 1; // TODO: use gridmap normal
            }
            // FIXME: cmd_ should be under robot frame
            // if (cmd_.linear.x != 0)
            //     robot_state_.moveDirection = atan2(cmd_.linear.y, cmd_.linear.x);
            robot_state_.moveDirection = body_state_.eular.yaw;
            // TODO: maxNormalForce, frictionMu
        }
    }

    void update_exp_path(void)
    {
        exp_path_.clear();
        // FIXME: pose.z is const?
        exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x, robot_state_.pose.y, robot_state_.pose.z));
        for (int i = 0; i < point_num_; ++i)
        {
            exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + (cmd_.linear.x * cos(robot_state_.pose.yaw) - cmd_.linear.y * sin(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                robot_state_.pose.y + (cmd_.linear.x * sin(robot_state_.pose.yaw) + cmd_.linear.y * cos(robot_state_.pose.yaw)) * multiply_factor_ * i,
                                                robot_state_.pose.z));
        }
    }

    void update_exp_path_test(void)
    {
        exp_path_.clear();
        // for (int i = 0; i < 10; ++i)
        // {
        //     exp_path_.push_back(Eigen::Vector3f(robot_state_.pose.x + 0.02 * i, robot_state_.pose.y + 0.02 * i, robot_state_.pose.z));
        // }
        // 初始化路径点
        std::vector<Eigen::Vector3f> pathPnts;
        // 添加点到 pathPnts
        pathPnts.emplace_back(0, 0, 0.2f); // 使用emplace_back而非push_back可以减少额外的拷贝构造开销
        // pathPnts.emplace_back(1, 0, 0.2f);
        // pathPnts.emplace_back(2, 0, 0.2f);
        // pathPnts.emplace_back(3, 0, 0.2f);
        // pathPnts.emplace_back(4, 0, 0.2f);
        // pathPnts.emplace_back(5, 0, 0.2f);
        // pathPnts.emplace_back(6, 0, 0.2f);

        pathPnts.emplace_back(-7.496126595059493525e-02, -1.409341165933408746e-02, 0.2f); // 使用emplace_back而非push_back可以减少额外的拷贝构造开销
        pathPnts.emplace_back(7.773619065856580690e-01, 8.947340691762608600e-01, 0.2f);
        pathPnts.emplace_back(1.579548421913896661e+00, 1.671847422354522550e+00, 0.2f);
        pathPnts.emplace_back(2.114339432132722685e+00, 1.645504596823055721e+00, 0.2f);
        pathPnts.emplace_back(2.440227703984819030e+00, 1.105476673427992829e+00, 0.2f);
        pathPnts.emplace_back(2.883101509322283817e+00, 7.761913542846619052e-01, 0.2f);
        pathPnts.emplace_back(3.367755862333094541e+00, 3.547061457811970797e-01, 0.2f);
        pathPnts.emplace_back(4.044600734641296214e+00, -1.409341165933408746e-02, 0.2f);
        pathPnts.emplace_back(4.788294483226851028e+00, 1.176207159979991701e-01, 0.2f);
        pathPnts.emplace_back(5.298017164841668958e+00, 3.151919074839977242e-01, 0.2f);
        pathPnts.emplace_back(5.582124889020420255e+00, 3.942203840783973234e-01, 0.2f);
        pathPnts.emplace_back(5.999930365753877837e+00, 7.103342904559957205e-01, 0.2f);
        pathPnts.emplace_back(6.451160280626011101e+00, 8.157055925818612607e-01, 0.2f);
        pathPnts.emplace_back(6.894034085963475889e+00, 1.408419167039857811e+00, 0.2f);
        pathPnts.emplace_back(7.244990686419580328e+00, 2.185532520218119501e+00, 0.2f);
        pathPnts.emplace_back(7.963616106401127936e+00, 3.265588367008247062e+00, 0.2f);

        // 线性插值，每两个点之间插值10个点
        for (size_t i = 0; i < pathPnts.size() - 1; ++i)
        {
            Eigen::Vector3f start = pathPnts[i];
            Eigen::Vector3f end = pathPnts[i + 1];

            for (int j = 0; j <= 20; ++j)
            {
                float t = static_cast<float>(j) / 20.0;
                Eigen::Vector3f interpolatedPoint = start + t * (end - start);
                exp_path_.push_back(interpolatedPoint);
            }
        }
    }

    void traj_planner()
    {
        double t = 0.0;
        double delta = 0.05;
        while (whole_body_planner_.get_state_traj_length() > 0)
        {
            MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
            pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(t);
            std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(t);
            for (size_t k = 0; k < 6; ++k)
            {
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }
            robot_interface_.pub_footcmd_from_footendpos(footend_interp);
            robot_interface_.pub_joint_state_from_footendpos(footend_interp);
            robot_interface_.pub_odom(odom_interp);
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                whole_body_planner_.dequeue_MCTsolution();
            }
            rate_.sleep();
        }
    }

    void run()
    {
        ros::spin();
    }
};