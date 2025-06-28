/**
 * @file UnitreeA1StateSequencePlanner.h
 * @author GitHub Copilot
 * @brief Unitree A1 State Sequence Planner with Trot Gait
 * @version 0.1
 * @date 2025-06-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include <pinocchio/math/rpy.hpp>
#include "legged_traj_plan/whole_body_planner/CmdVelExtrapolator.h"
#include "legged_traj_plan/whole_body_planner/A1StateSequencePlanner.h"
#include "UnitreeA1PlannerBase.h"

#include "legged_traj_plan/A1_State.h"
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

// Simple trot gait patterns for A1 (FR, FL, RR, RL)
enum class TrotPhase
{
    PHASE_1, // FR+RL swing, FL+RR stance
    PHASE_2  // FL+RR swing, FR+RL stance
};

// Raibert heuristic for foothold placement
struct RaibertFootholdPlanner
{
    double step_time;
    double stance_time;
    Eigen::Vector3d velocity_gain;

    RaibertFootholdPlanner(double step_t = 0.4, double stance_t = 0.2)
        : step_time(step_t), stance_time(stance_t), velocity_gain(0.5, 0.5, 0.0) {}

    Eigen::Vector3d computeFoothold(const pinocchio::SE3 &body_pose,
                                    const Eigen::Vector3d &body_velocity,
                                    const Eigen::Vector3d &nominal_foothold,
                                    int leg_index)
    {
        // Raibert heuristic: foothold = nominal + velocity_gain * body_velocity * (step_time/2 + stance_time/2)
        double foothold_time = step_time / 2.0 + stance_time / 2.0;
        Eigen::Vector3d velocity_offset = velocity_gain.cwiseProduct(body_velocity) * foothold_time;

        // Transform nominal foothold to world frame
        Eigen::Vector3d world_nominal = point_SE3Act(body_pose, nominal_foothold);

        // Add velocity-based offset
        Eigen::Vector3d target_foothold = world_nominal + velocity_offset;

        // Transform back to body frame
        return point_SE3Act(body_pose.inverse(), target_foothold);
    }
};

// For robot state recording
struct A1RobotProfile
{
    double time;
    double t; // param time in traj
    pinocchio::SE3 pose;
    PosList foot_pos_list;
    PosList cfg_pos_list;
    PosList cfg_vel_list;
    std::array<double, 4> foot_end_sdf; // 4 legs for A1
    std::array<bool, 4> support_state;  // 4 legs for A1
    geometry_msgs::Twist cmd_vel;
};

struct UnitreeA1StateSequencePlannerConfig
{
    int rosRate;
    double stepTime;
    double trotStepDuration;
    double trotStanceDuration;

    bool enableReachableCheck;
    int reachableCheckSize;

    bool swingTrajPreOpt;
    bool shutdownAfterPreOpt;
    bool execOnKeyboardCmd;

    std::string demoPath;
    bool savePlannedStates;
    bool execSavedStates;

    std::string OptBenchmarkSavePath;

    void loadParams(ros::NodeHandle &nh, std::string ns = "A1StateSequencePlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/rosRate", rosRate);
        check_digit &= nh.getParam(ns + "/stepTime", stepTime);
        check_digit &= nh.getParam(ns + "/trotStepDuration", trotStepDuration);
        check_digit &= nh.getParam(ns + "/trotStanceDuration", trotStanceDuration);

        check_digit &= nh.getParam(ns + "/enableReachableCheck", enableReachableCheck);
        check_digit &= nh.getParam(ns + "/reachableCheckSize", reachableCheckSize);

        check_digit &= nh.getParam(ns + "/swingTrajPreOpt", swingTrajPreOpt);
        check_digit &= nh.getParam(ns + "/shutdownAfterPreOpt", shutdownAfterPreOpt);
        check_digit &= nh.getParam(ns + "/execOnKeyboardCmd", execOnKeyboardCmd);

        check_digit &= nh.getParam(ns + "/demoPath", demoPath);
        check_digit &= nh.getParam(ns + "/savePlannedStates", savePlannedStates);
        check_digit &= nh.getParam(ns + "/execSavedStates", execSavedStates);

        check_digit &= nh.getParam(ns + "/OptBenchmarkSavePath", OptBenchmarkSavePath);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load UnitreeA1StateSequencePlannerConfig.");
        }
    }
};

class UnitreeA1StateSequencePlanner : public UnitreeA1PlannerBase
{
private:
    ros::Rate rate_;

    UnitreeA1StateSequencePlannerConfig config_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    // Interface
    A1StateSequencePlanner a1_state_sequence_planner_;

    // Trot gait state
    TrotPhase current_phase_;
    RaibertFootholdPlanner raibert_planner_;

    // Status
    bool motion_lock_ = false;
    Eigen::Vector3d body_velocity_;

    // Benchmarking
    double init_time_ = 0.0;
    std::vector<A1RobotProfile> robot_profile_;
    Benchmark benchmark_;
    ros::Publisher benchmark_progress_pub_;

    // Visualizer
    GCSVisualizer visualizer_;
    GCSVisualizer visualizer_base_;

public:
    UnitreeA1StateSequencePlanner() : UnitreeA1PlannerBase(),
                                      a1_state_sequence_planner_(swing_traj_planner_, gridmap_interface_, robot_interface_),
                                      visualizer_(nh_, "odom", "a1_visualizer_markers"),
                                      visualizer_base_(nh_, "base", "a1_visualizer_markers_base"),
                                      rate_(100),
                                      benchmark_("UnitreeA1StateSequencePlannerBenchmark", swing_traj_planner_config_.enableBenchmark),
                                      current_phase_(TrotPhase::PHASE_1),
                                      raibert_planner_(0.4, 0.2)
    {
        config_.loadParams(nh_);
        rate_ = ros::Rate(config_.rosRate);
        init_time_ = ros::Time::now().toSec();
        benchmark_progress_pub_ = nh_.advertise<std_msgs::Bool>("/benchmark_progress", 1);
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &UnitreeA1StateSequencePlanner::cmd_callback, this);
        a1_state_sequence_planner_.enableRecordStates(config_.savePlannedStates);

        body_velocity_ = Eigen::Vector3d::Zero();
    }

    // Trot gait generation
    A1_State generateNextTrotState(const A1_State &current_state, const geometry_msgs::Twist &cmd_vel)
    {
        A1_State next_state = current_state;

        // Update body pose based on command velocity
        pinocchio::SE3 current_pose = XYZRPY2SE3(current_state.base_Pose_Now);

        // Simple velocity integration
        double dt = config_.trotStepDuration;
        Eigen::Vector3d velocity(cmd_vel.linear.x, cmd_vel.linear.y, 0.0);
        Eigen::Vector3d angular_velocity(0, 0, cmd_vel.angular.z);

        // Update position
        current_pose.translation() += velocity * dt;

        // Update orientation (simple yaw rotation)
        Eigen::Vector3d current_rpy = pinocchio::rpy::matrixToRpy(current_pose.rotation());
        current_rpy[2] += angular_velocity[2] * dt;
        current_pose.rotation() = pinocchio::rpy::rpyToMatrix(current_rpy);

        next_state.base_Pose_Now = SE32XYZRPY(current_pose);

        // Set contact pattern based on current trot phase
        std::array<bool, 4> contact_pattern = getTrotContactPattern(current_phase_);
        for (int i = 0; i < 4; i++)
        {
            next_state.support_State_Now[i] = contact_pattern[i];
        }

        // Update foot positions using Raibert heuristic for swing legs
        for (int i = 0; i < 4; i++)
        {
            if (!contact_pattern[i]) // Swing leg
            {
                Eigen::Vector3d nominal_foothold = robot_interface_->getNominalFoothold(i);
                Eigen::Vector3d target_foothold = raibert_planner_.computeFoothold(
                    current_pose, velocity, nominal_foothold, i);

                // Set target foothold in world frame
                Eigen::Vector3d world_foothold = point_SE3Act(current_pose, target_foothold);
                next_state.feetPositionNow.foot[i].x = world_foothold[0];
                next_state.feetPositionNow.foot[i].y = world_foothold[1];
                next_state.feetPositionNow.foot[i].z = world_foothold[2];
            }
            // Stance legs keep their position (no update needed)
        }

        return next_state;
    }

    // State conversion utilities
    A1_State getCurrentA1State()
    {
        A1_State state;

        // Get current body pose
        pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
        state.base_Pose_Now = SE32XYZRPY(body_pose);
        state.base_Pose_Next = state.base_Pose_Now;

        // Get current foot positions
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        for (int i = 0; i < 4; i++)
        {
            state.feetPositionNow.foot[i] = foot_state.position[i];
            state.support_State_Now[i] = foot_state.contact[i];
            state.faultLeg_State_Now[i] = 0; // Normal
        }
        state.feetPositionNext = state.feetPositionNow;
        state.support_State_Next = state.support_State_Now;
        state.faultLeg_State_Next = state.faultLeg_State_Now;

        // Set move direction based on current velocity
        state.move_Direction.x = body_velocity_[0];
        state.move_Direction.y = body_velocity_[1];
        state.move_Direction.z = 0.0;

        return state;
    }

    void updateBodyVelocity()
    {
        // Simple velocity estimation based on command
        body_velocity_ = Eigen::Vector3d(cmd_.linear.x, cmd_.linear.y, 0.0);
    }

    // Trot pattern helpers
    std::array<bool, 4> getTrotContactPattern(TrotPhase phase)
    {
        std::array<bool, 4> pattern;
        if (phase == TrotPhase::PHASE_1)
        {
            // FR+RL swing (false), FL+RR stance (true)
            pattern[0] = false; // FR
            pattern[1] = true;  // FL
            pattern[2] = false; // RR
            pattern[3] = true;  // RL
        }
        else // PHASE_2
        {
            // FL+RR swing (false), FR+RL stance (true)
            pattern[0] = true;  // FR
            pattern[1] = false; // FL
            pattern[2] = true;  // RR
            pattern[3] = false; // RL
        }
        return pattern;
    }

    void switchTrotPhase()
    {
        current_phase_ = (current_phase_ == TrotPhase::PHASE_1) ? TrotPhase::PHASE_2 : TrotPhase::PHASE_1;
    }

    // Contact handling
    bool is_contact(int leg_idx, double eps = 0.1)
    {
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        Eigen::Vector3d foot_force(foot_state.effort[leg_idx].x,
                                   foot_state.effort[leg_idx].y,
                                   foot_state.effort[leg_idx].z);
        return (foot_force.norm() > eps) || foot_state.contact[leg_idx];
    }

    void stance_contact_handle()
    {
        bool flag = false;
        int max_cnt = 200;
        double alpha = 0.005;
        ros::spinOnce();

        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        std::vector<Eigen::Vector3d> footend_interp(4, Eigen::Vector3d::Zero());
        for (size_t k = 0; k < 4; ++k)
        {
            footend_interp.at(k)[0] = foot_state.position[k].x;
            footend_interp.at(k)[1] = foot_state.position[k].y;
            footend_interp.at(k)[2] = foot_state.position[k].z;
        }

        ROS_INFO("A1 stance contact handling...");
        while (!flag && max_cnt-- > 0)
        {
            flag = true;
            for (size_t k = 0; k < 4; ++k)
            {
                if (is_contact(k) == false)
                {
                    flag = false;
                    footend_interp.at(k) = A1_NOMINAL_FOOT_POS[k] * alpha + footend_interp.at(k) * (1 - alpha);
                }
            }
            robot_interface_->setJointCmd(robot_interface_->IKFast_foots(footend_interp));
            ros::spinOnce();
            rate_.sleep();
        }
        if (max_cnt <= 0)
            ROS_WARN("A1 stance contact handling failed.");
        else
            ROS_INFO("A1 stance contact handling done.");
    }

    // Planning
    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }

    void traj_planner()
    {
        if (config_.swingTrajPreOpt)
        {
            benchmark_.reset();
            a1_state_sequence_planner_.optSwingTraj();
            benchmark_.record("SwingTrajOptimization");
            a1_state_sequence_planner_.reachableCheck();
            benchmark_.record("ReachabilityCheck");
            benchmark_.end();

            if (config_.shutdownAfterPreOpt)
            {
                std_msgs::Bool msg;
                msg.data = true;
                benchmark_progress_pub_.publish(msg);
                ros::shutdown();
                return;
            }
        }

        double t = 0.0;
        double delta = 1 / config_.stepTime / config_.rosRate;

        // Auto opt next traj before exec
        A1StateTransfer &state_traj = a1_state_sequence_planner_.get_state_traj(0);
        pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp_vel = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
        std::vector<Eigen::Vector3d> footend_interp_acc = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
        std::array<bool, 4> support_state = state_traj.eval_support_state(0.5);

        if (config_.enableReachableCheck)
            for (int i = 0; i < 4; i++)
                if (support_state[i] == false)
                    state_traj.reachable_check(i, config_.reachableCheckSize, 0.75 / config_.reachableCheckSize);

        do
        {
            // Get Interpolated State
            odom_interp = state_traj.eval_torso_traj(sine_remap(t));
            support_state = state_traj.eval_support_state(sine_remap(t));
            std::vector<bool> contact = std::vector<bool>(support_state.begin(), support_state.end());

            // Configuration space command (joint angles)
            footend_interp = state_traj.eval_cfg_traj(sine_remap(t));
            robot_interface_->setJointCmd(footend_interp, contact);
            robot_interface_->setBodyPoseCmd(odom_interp);

            // Wait Key
            if (t == 0.0)
                waitKey("Press any key with Enter to execute A1 trajectory.");

            // State recording
            A1RobotProfile profile;
            profile.time = ros::Time::now().toSec() - init_time_;
            profile.t = t;
            profile.pose = odom_interp;
            profile.foot_pos_list = state_traj.eval_foot_traj(sine_remap(t));
            profile.cfg_pos_list = state_traj.eval_cfg_traj(sine_remap(t), 0, false);
            profile.cfg_vel_list = state_traj.eval_cfg_traj(sine_remap(t), 1, false);
            profile.support_state = support_state;
            profile.cmd_vel = cmd_;
            for (size_t k = 0; k < 4; ++k)
            {
                profile.foot_end_sdf[k] = gridmap_interface_->sdfValue(profile.foot_pos_list[k]);
            }
            robot_profile_.emplace_back(profile);

            // Update param t
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                a1_state_sequence_planner_.dequeue_A1solution();
                switchTrotPhase(); // Switch to next trot phase

                if (a1_state_sequence_planner_.get_state_traj_length() > 0)
                {
                    a1_state_sequence_planner_.visClear();
                    ros::Duration(0.2).sleep();
                    state_traj = a1_state_sequence_planner_.get_state_traj(0);
                    support_state = state_traj.eval_support_state(0.5);
                    if (config_.enableReachableCheck)
                        for (int i = 0; i < 4; i++)
                            if (support_state[i] == false)
                                state_traj.reachable_check(i, config_.reachableCheckSize, 0.75 / config_.reachableCheckSize);
                }
            }
            ros::spinOnce();
            rate_.sleep();
        } while (a1_state_sequence_planner_.get_state_traj_length() > 0 && ros::ok());

        // Set all foot contact to true
        robot_interface_->setJointCmd(footend_interp, std::vector<bool>(4, true));

        // Stance contact handling
        stance_contact_handle();
    }

    // Cmd callbacks
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        if (motion_lock_)
        {
            ROS_WARN("A1 robot is in motion, ignore new command.");
            return;
        }

        motion_lock_ = true;
        gridmap_interface_->lockMapUpdate();
        cmd_ = msg;
        updateBodyVelocity();
        a1_state_sequence_planner_.visClear();

        // Generate next trot state using Raibert heuristic
        A1_State current_state = getCurrentA1State();
        A1_State next_state = generateNextTrotState(current_state, cmd_);

        // Visualization
        visualizer_.delAll();
        visualizer_base_.delAll();

        // Enqueue the trot transition
        bool ret = a1_state_sequence_planner_.enqueue_A1solution(current_state, next_state);

        if (ret)
        {
            ROS_INFO("A1 trot gait planned successfully.");
            traj_planner();
        }
        else
        {
            ROS_WARN("A1 trot gait planning failed.");
        }

        motion_lock_ = false;
        gridmap_interface_->unlockMapUpdate();
    }

    // Benchmarking
    void saveRobotProfile()
    {
        std::ofstream file(config_.OptBenchmarkSavePath + "/a1_robot_profile.csv");
        if (file.is_open())
        {
            file << "time,t,pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,";
            file << "foot0_x,foot0_y,foot0_z,foot1_x,foot1_y,foot1_z,foot2_x,foot2_y,foot2_z,foot3_x,foot3_y,foot3_z,";
            file << "cfg0_x,cfg0_y,cfg0_z,cfg1_x,cfg1_y,cfg1_z,cfg2_x,cfg2_y,cfg2_z,cfg3_x,cfg3_y,cfg3_z,";
            file << "cfg0_dx,cfg0_dy,cfg0_dz,cfg1_dx,cfg1_dy,cfg1_dz,cfg2_dx,cfg2_dy,cfg2_dz,cfg3_dx,cfg3_dy,cfg3_dz,";
            file << "foot0_sdf,foot1_sdf,foot2_sdf,foot3_sdf,";
            file << "support0,support1,support2,support3,";
            file << "cmd_vel_x,cmd_vel_y,cmd_vel_z\n";

            for (const auto &profile : robot_profile_)
            {
                file << profile.time << "," << profile.t << ",";
                auto pos = profile.pose.translation();
                file << pos[0] << "," << pos[1] << "," << pos[2] << ",";
                auto rpy = profile.pose.rotation().eulerAngles(0, 1, 2);
                file << rpy[0] << "," << rpy[1] << "," << rpy[2] << ",";
                for (size_t k = 0; k < 4; ++k)
                {
                    file << profile.foot_pos_list[k][0] << "," << profile.foot_pos_list[k][1] << "," << profile.foot_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 4; ++k)
                {
                    file << profile.cfg_pos_list[k][0] << "," << profile.cfg_pos_list[k][1] << "," << profile.cfg_pos_list[k][2] << ",";
                }
                for (size_t k = 0; k < 4; ++k)
                {
                    file << profile.cfg_vel_list[k][0] << "," << profile.cfg_vel_list[k][1] << "," << profile.cfg_vel_list[k][2] << ",";
                }
                for (size_t k = 0; k < 4; ++k)
                {
                    file << profile.foot_end_sdf[k] << ",";
                }
                for (size_t k = 0; k < 4; ++k)
                {
                    file << profile.support_state[k] << ",";
                }
                file << profile.cmd_vel.linear.x << "," << profile.cmd_vel.linear.y << "," << profile.cmd_vel.angular.z << "\n";
            }
        }
        file.close();
    }

    // Utilities
    char waitKey(std::string info = "Press any key with Enter to continue.")
    {
        if (config_.execOnKeyboardCmd)
        {
            std::cout << info << std::endl;
            return getchar();
        }
        return 0;
    }

    void run()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            rate_.sleep();
        }

        if (swing_traj_planner_config_.enableBenchmark)
        {
            std::cout << "Save A1 benchmark results..." << std::endl;
            a1_state_sequence_planner_.saveBenchmarkResults();
            benchmark_.save(config_.OptBenchmarkSavePath + "/a1_benchmark.csv");
            std::cout << "Save A1 robot profile..." << std::endl;
            saveRobotProfile();
            std::cout << "A1 benchmark results and robot profile saved." << std::endl;
        }
    }
};