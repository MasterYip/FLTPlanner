/**
 * @file ElSpiderAirRaibertPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include <pinocchio/math/rpy.hpp>
#include "legged_traj_plan/whole_body_planner/CmdVelExtrapolator.h"
#include "legged_traj_plan/whole_body_planner/RaibertHeuristicPlanner.h"
#include "ElSpiderAirPlannerBase.h"

#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/BodyState.h"

/* external project header files */
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "legged_traj_search/utils/gcs_visualizer.hpp"

// For robot state recording
struct RobotProfile
{
    double time;
    double t; // param time in traj
    pinocchio::SE3 pose;
    PosList foot_pos_list;
    PosList cfg_pos_list;
    PosList cfg_vel_list;
    std::array<double, 6> foot_end_sdf;
    std::array<bool, 6> support_state;
    geometry_msgs::Twist cmd_vel;
};

struct ElSpiderAirRaibertPlannerConfig
{
    int rosRate;

    std::string demoPath;

    std::string benchmarkSavePath;

    void loadParams(ros::NodeHandle &nh, std::string ns = "RaibertPlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/rosRate", rosRate);

        check_digit &= nh.getParam(ns + "/demoPath", demoPath);

        check_digit &= nh.getParam(ns + "/benchmarkSavePath", benchmarkSavePath);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load ElSpiderAirRaibertPlannerConfig.");
        }
    }
};

class ElSpiderAirRaibertPlanner : public ElSpiderAirPlannerBase
{
private:
    ros::Rate rate_;

    ElSpiderAirRaibertPlannerConfig config_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    // Interface
    RaibertHeuristicPlanner raibert_planner_;

    ros::Timer timer_;
    // Benchmarking
    double init_time_ = 0.0;
    std::vector<RobotProfile> robot_profile_;
    Benchmark benchmark_;

    // Visualizer
    GCSVisualizer visualizer_;
    GCSVisualizer visualizer_base_;

public:
    ElSpiderAirRaibertPlanner() : ElSpiderAirPlannerBase(),
                                  raibert_planner_(swing_traj_planner_, gridmap_interface_, robot_interface_),
                                  visualizer_(nh_, "odom", "visualizer_markers"),
                                  visualizer_base_(nh_, "base", "visualizer_markers_base"),
                                  rate_(100), benchmark_("ElSpiderAirRaibertPlannerBenchmark",
                                                         swing_traj_planner_config_.enableBenchmark)
    {
        config_.loadParams(nh_);
        rate_ = ros::Rate(config_.rosRate);
        init_time_ = ros::Time::now().toSec();
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &ElSpiderAirRaibertPlanner::cmd_callback, this);

        timer_ = nh_.createTimer(ros::Duration(0.01), &ElSpiderAirRaibertPlanner::timer_callback, this);
        raibert_planner_.start(robot_interface_->getBodyPoseFdb(), getFootPos());
    }

    PosList getFootPos()
    {
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        PosList foot_pos_list;
        for (size_t i = 0; i < 6; ++i)
        {
            foot_pos_list.push_back(Eigen::Vector3d(foot_state.position[i].x, foot_state.position[i].y, foot_state.position[i].z));
        }
        return foot_pos_list;
    }

    void timer_callback(const ros::TimerEvent &event)
    {
        pinocchio::SE3 exp_pose;
        PosList exp_foot_pos;
        std::array<bool, 6> contact_state;
        if (swing_traj_planner_config_.useCfgCommand)
        {
            if (raibert_planner_.queryCfg(ros::Time::now().toSec(), exp_pose, exp_foot_pos, contact_state))
            {
                robot_interface_->setBodyPoseCmd(exp_pose);
                robot_interface_shadow_->setBodyPoseCmd(exp_pose);
                robot_interface_->setJointCmd(exp_foot_pos,
                                             PosList(6, Eigen::Vector3d::Zero()),
                                             PosList(6, Eigen::Vector3d::Zero()),
                                             std::vector<bool>(contact_state.begin(), contact_state.end()));
                robot_interface_shadow_->setJointCmd(exp_foot_pos,
                                                    PosList(6, Eigen::Vector3d::Zero()),
                                                    PosList(6, Eigen::Vector3d::Zero()),
                                                    std::vector<bool>(contact_state.begin(), contact_state.end()));
            }
        }
        else
        {
            if (raibert_planner_.query(ros::Time::now().toSec(), exp_pose, exp_foot_pos, contact_state))
            {
                robot_interface_->setBodyPoseCmd(exp_pose);
                robot_interface_shadow_->setBodyPoseCmd(exp_pose);
                robot_interface_->setFootCmd(exp_foot_pos,
                                             PosList(6, Eigen::Vector3d::Zero()),
                                             PosList(6, Eigen::Vector3d::Zero()),
                                             std::vector<bool>(contact_state.begin(), contact_state.end()));
                robot_interface_shadow_->setFootCmd(exp_foot_pos,
                                                    PosList(6, Eigen::Vector3d::Zero()),
                                                    PosList(6, Eigen::Vector3d::Zero()),
                                                    std::vector<bool>(contact_state.begin(), contact_state.end()));
            }
        }
    }

    // cmd_vel callback
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("cmd_vel received");
        cmd_ = msg;
        raibert_planner_.update(robot_interface_->getBodyPoseFdb(), cmd_);
    }

    //// Contact Handling (Sim only)
    // FIXME: avoid error detection
    bool is_contact(int leg_idx, double eps = 0.1)
    {
        Eigen::Vector3d foot_force;
        auto foot_state_ = robot_interface_->getFootStateFdb();
        foot_force << foot_state_.effort[leg_idx].x, foot_state_.effort[leg_idx].y, foot_state_.effort[leg_idx].z;
        return (foot_force.norm() > eps) || foot_state_.contact[leg_idx];
    }

    //// Planning
    // for lift & touch smoothing
    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }

    void saveRobotProfile()
    {
        // Save to file
        std::ofstream file(swing_traj_planner_config_.robotProfilePath);
        if (file.is_open())
        {
            file << "time,t,pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,";
            file << "foot0_x,foot0_y,foot0_z,foot1_x,foot1_y,foot1_z,foot2_x,foot2_y,foot2_z,";
            file << "foot3_x,foot3_y,foot3_z,foot4_x,foot4_y,foot4_z,foot5_x,foot5_y,foot5_z,";
            file << "cfg0_x,cfg0_y,cfg0_z,cfg1_x,cfg1_y,cfg1_z,cfg2_x,cfg2_y,cfg2_z,";
            file << "cfg3_x,cfg3_y,cfg3_z,cfg4_x,cfg4_y,cfg4_z,cfg5_x,cfg5_y,cfg5_z,";
            file << "cfg0_dx,cfg0_dy,cfg0_dz,cfg1_dx,cfg1_dy,cfg1_dz,cfg2_dx,cfg2_dy,cfg2_dz,";
            file << "cfg3_dx,cfg3_dy,cfg3_dz,cfg4_dx,cfg4_dy,cfg4_dz,cfg5_dx,cfg5_dy,cfg5_dz,";
            file << "foot0_sdf,foot1_sdf,foot2_sdf,foot3_sdf,foot4_sdf,foot5_sdf,";
            file << "support0,support1,support2,support3,support4,support5\n";
            for (const auto &profile : robot_profile_)
            {
                file << profile.time << "," << profile.t << ",";
                auto pos = profile.pose.translation();
                file << pos[0] << "," << pos[1] << "," << pos[2] << ",";
                auto rpy = profile.pose.rotation().eulerAngles(0, 1, 2);
                file << rpy[0] << "," << rpy[1] << "," << rpy[2] << ",";
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.foot_pos_list[k][0] << "," << profile.foot_pos_list[k][1] << "," << profile.foot_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.cfg_pos_list[k][0] << "," << profile.cfg_pos_list[k][1] << "," << profile.cfg_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.cfg_vel_list[k][0] << "," << profile.cfg_vel_list[k][1] << "," << profile.cfg_vel_list[k][2] << ",";
                }
                for (size_t k = 0; k < 6; ++k)
                {
                    file << profile.foot_end_sdf[k] << ",";
                }
                for (size_t k = 0; k < 5; ++k)
                {
                    file << profile.support_state[k] << ",";
                }
                file << profile.support_state[5] << "\n";
            }
        }
        file.close();
    }

    void run()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
        if (swing_traj_planner_config_.enableBenchmark)
        {
            std::cout << "Save benchmark results..." << std::endl;
            raibert_planner_.saveBenchmarkResults();
            benchmark_.save(config_.benchmarkSavePath);
            std::cout << "Save robot profile..." << std::endl;
            saveRobotProfile();
            std::cout << "Benchmark results and robot profile saved." << std::endl;
        }
    }
};