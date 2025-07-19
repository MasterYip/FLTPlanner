/**
 * @file Hexapod201StateSequencePlanner.h
 * @author GitHub Copilot
 * @brief State sequence planner for Hexapod201 robot
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
#include "legged_traj_plan/whole_body_planner/StateSequencePlanner.h"
#include "../elspider_air_planner/ElSpiderAirPlannerBase.h"

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

// Simple tripod gait patterns for hexapod (legs 0-5: RF, RR, FL, LF, LR, RL)
enum class TripodPhase
{
    PHASE_135, // Legs 1,3,5 (RR, LF, RL) swing, Legs 0,2,4 (RF, FL, LR) stance
    PHASE_246  // Legs 0,2,4 (RF, FL, LR) swing, Legs 1,3,5 (RR, LF, RL) stance
};

// Raibert heuristic for hexapod foothold placement
struct Hexapod201RaibertFootholdPlanner
{
    double step_time;
    double stance_time;
    Eigen::Vector3d velocity_gain;

    Hexapod201RaibertFootholdPlanner(double step_t = 0.4, double stance_t = 0.2)
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
        Eigen::Vector3d world_nominal = point_SE3Act(body_pose.inverse(), nominal_foothold);

        // Add velocity-based offset for forward motion
        return world_nominal + velocity_offset;
    }
};

// For robot state recording
struct RobotProfile
{
    double time;
    double t; // param time in traj
    pinocchio::SE3 pose;
    PosList foot_pos_list;
    std::array<double, 6> foot_end_sdf;
    std::array<bool, 6> support_state;
    geometry_msgs::Twist cmd_vel;
};

struct Hexapod201StateSequencePlannerConfig
{
    int rosRate;
    double stepTime;

    bool execOnKeyboardCmd;

    // Tripod gait parameters
    double tripodStepDuration;
    double tripodStanceDuration;

    void loadParams(ros::NodeHandle &nh, std::string ns = "StateSequencePlanner")
    {
        bool check_digit = true;
        check_digit &= nh.getParam(ns + "/rosRate", rosRate);
        check_digit &= nh.getParam(ns + "/stepTime", stepTime);
        check_digit &= nh.getParam(ns + "/execOnKeyboardCmd", execOnKeyboardCmd);
        check_digit &= nh.getParam(ns + "/tripodStepDuration", tripodStepDuration);
        check_digit &= nh.getParam(ns + "/tripodStanceDuration", tripodStanceDuration);
        if (!check_digit)
        {
            ROS_ERROR("Failed to load Hexapod201StateSequencePlannerConfig.");
        }
    }
};

class Hexapod201StateSequencePlanner : public ElSpiderAirPlannerBase
{
private:
    ros::Rate rate_;

    Hexapod201StateSequencePlannerConfig config_;

    // Cmd
    ros::Subscriber cmd_sub_;
    geometry_msgs::Twist cmd_;

    // Interface
    StateSequencePlanner state_sequence_planner_;

    GridMapCmdVelExtrapolator gridmap_extrapolator_;

    // Status
    bool motion_lock_ = false;

    // Benchmarking
    double init_time_ = 0.0;
    std::vector<RobotProfile> robot_profile_;

    // Visualizer
    GCSVisualizer visualizer_;

    // Tripod gait state
    TripodPhase current_tripod_phase_;
    Hexapod201RaibertFootholdPlanner hexapod_raibert_planner_;

public:
    Hexapod201StateSequencePlanner() : ElSpiderAirPlannerBase(),
                                        state_sequence_planner_(swing_traj_planner_, gridmap_interface_, robot_interface_),
                                        visualizer_(nh_, "odom", "visualizer_markers"),
                                        rate_(100),
                                        current_tripod_phase_(TripodPhase::PHASE_135),
                                        hexapod_raibert_planner_(0.4, 0.2)
    {
        config_.loadParams(nh_);
        rate_ = ros::Rate(config_.rosRate);
        init_time_ = ros::Time::now().toSec();
        cmd_sub_ = nh_.subscribe("/cmd_vel", 1, &Hexapod201StateSequencePlanner::cmd_callback, this);

        PosList pose_sample_pts;
        int len = 6;
        double delta = 0.2;
        for (int i = 0; i < len; i++)
        {
            for (int j = 0; j < len; j++)
            {
                pose_sample_pts.emplace_back(Eigen::Vector3d(i * delta - 0.5 * len * delta,
                                                             j * delta - 0.5 * len * delta, 0));
            }
        }
        gridmap_extrapolator_.init(gridmap_interface_, pose_sample_pts);
    }

    // cmd_vel callback
    void cmd_callback(const geometry_msgs::Twist &msg)
    {
        if (motion_lock_)
            ROS_WARN("Robot is in motion, ignore new command.");
        else
        {
            motion_lock_ = true;
            gridmap_interface_->lockMapUpdate();
            cmd_ = msg;
            state_sequence_planner_.visClear();

            // Fetch feedback
            ros::spinOnce();
            
            // Use simple tripod gait planner (Raibert-style)
            legged_traj_plan::hexapod_State current_state = getCurrentHexapodState();
            legged_traj_plan::hexapod_State next_state = generateNextTripodState(current_state, cmd_);
            
            bool ret = state_sequence_planner_.enqueue_MCTsolution(current_state, next_state);
            
            if (ret)
            {
                ROS_INFO("Hexapod201 tripod gait planned successfully.");
            }
            else
            {
                ROS_WARN("Hexapod201 tripod gait planning failed.");
            }

            // Visualization
            visualizer_.delAll();

            if (swing_traj_planner_config_.enableVis)
            {
                // Vis current robot state
                visualizer_.visSphere(Point3D(current_state.base_Pose_Now.position.x, 
                                           current_state.base_Pose_Now.position.y, 
                                           current_state.base_Pose_Now.position.z));
            }

            traj_planner();
            motion_lock_ = false;
            gridmap_interface_->unlockMapUpdate();
        }
    }

    // Get current robot state and convert to hexapod_State
    legged_traj_plan::hexapod_State getCurrentHexapodState()
    {
        legged_traj_plan::hexapod_State hexapodState;
        
        // Get body pose
        pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
        hexapodState.base_Pose_Now.position.x = body_pose.translation()[0];
        hexapodState.base_Pose_Now.position.y = body_pose.translation()[1];
        hexapodState.base_Pose_Now.position.z = body_pose.translation()[2];
        
        Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(body_pose.rotation());
        hexapodState.base_Pose_Now.orientation.roll = rpy[0];
        hexapodState.base_Pose_Now.orientation.pitch = rpy[1];
        hexapodState.base_Pose_Now.orientation.yaw = rpy[2];
        hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;

        // Get foot states
        legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
        for (int i = 0; i < 6; i++)
        {
            hexapodState.feetPositionNow.foot[i].x = foot_state.position[i].x;
            hexapodState.feetPositionNow.foot[i].y = foot_state.position[i].y;
            hexapodState.feetPositionNow.foot[i].z = foot_state.position[i].z;
            hexapodState.support_State_Now[i] = true; // Default all stance
            hexapodState.faultLeg_State_Now[i] = 0; // Normal
        }
        
        hexapodState.move_Direction.x = cos(rpy[2]);
        hexapodState.move_Direction.y = sin(rpy[2]);
        hexapodState.move_Direction.z = 0;
        
        // 下一步落足点
        hexapodState.feetPositionNext = hexapodState.feetPositionNow;
        // 下一步支撑状态和容错状态
        hexapodState.support_State_Next = hexapodState.support_State_Now;
        hexapodState.faultLeg_State_Next = hexapodState.faultLeg_State_Now;
        return hexapodState;
    }

    // Tripod gait generation
    legged_traj_plan::hexapod_State generateNextTripodState(const legged_traj_plan::hexapod_State &current_state, 
                                                           const geometry_msgs::Twist &cmd_vel)
    {
        legged_traj_plan::hexapod_State next_state = current_state;

        // Get current pose and update the extrapolator
        pinocchio::SE3 current_pose = XYZRPY2SE3(current_state.base_Pose_Now);
        gridmap_extrapolator_.update(current_pose, cmd_vel);

        // Use terrain-aware pose extrapolation
        double dt = config_.tripodStepDuration;
        pinocchio::SE3 next_pose = gridmap_extrapolator_.extrapolate(dt);

        next_state.base_Pose_Now = SE32XYZRPY(next_pose);

        // Set contact pattern based on current tripod phase
        std::array<bool, 6> contact_pattern = getTripodContactPattern(current_tripod_phase_);
        for (int i = 0; i < 6; i++)
        {
            next_state.support_State_Now[i] = contact_pattern[i];
        }

        // Update foot positions using Raibert heuristic for swing legs
        Eigen::Vector3d velocity(cmd_vel.linear.x, cmd_vel.linear.y, 0.0);
        for (int i = 0; i < 6; i++)
        {
            if (!contact_pattern[i]) // Swing leg
            {
                Eigen::Vector3d nominal_foothold = robot_interface_->getNominalFoothold(i);
                Eigen::Vector3d target_foothold = hexapod_raibert_planner_.computeFoothold(
                    next_pose, velocity, nominal_foothold, i);

                // Set target foothold in world frame with terrain-aware height
                next_state.feetPositionNow.foot[i].x = target_foothold[0];
                next_state.feetPositionNow.foot[i].y = target_foothold[1];
                next_state.feetPositionNow.foot[i].z = gridmap_interface_->value(
                    grid_map::Position(target_foothold[0], target_foothold[1]));
            }
        }

        return next_state;
    }

    // Tripod pattern helpers
    std::array<bool, 6> getTripodContactPattern(TripodPhase phase)
    {
        std::array<bool, 6> pattern;
        if (phase == TripodPhase::PHASE_135)
        {
            pattern[0] = true;  pattern[1] = false; pattern[2] = true;
            pattern[3] = false; pattern[4] = true;  pattern[5] = false;
        }
        else // PHASE_246
        {
            pattern[0] = false; pattern[1] = true;  pattern[2] = false;
            pattern[3] = true;  pattern[4] = false; pattern[5] = true;
        }
        return pattern;
    }

    void switchTripodPhase()
    {
        current_tripod_phase_ = (current_tripod_phase_ == TripodPhase::PHASE_135) ? 
                               TripodPhase::PHASE_246 : TripodPhase::PHASE_135;
    }

    // Simplified trajectory planner - only uses setBodyPoseCmd and setFootCmd
    void traj_planner()
    {
        double t = 0.0;
        double delta = 1 / config_.stepTime / config_.rosRate;

        MCTStateTransfer &state_traj = state_sequence_planner_.get_state_traj(0);
        pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
        std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(0.0);
        std::array<bool, 6> support_state = state_traj.eval_support_state(0.5);

        do
        {
            // Get Interpolated State
            odom_interp = state_traj.eval_torso_traj(sine_remap(t));
            support_state = state_traj.eval_support_state(sine_remap(t));

            // Footend position in world frame - simplified for Hexapod201
            footend_interp = state_traj.eval_foot_traj(sine_remap(t));

            // Only use setBodyPoseCmd and setFootCmd (no kinematics)
            robot_interface_->setBodyPoseCmd(odom_interp);
            robot_interface_->setFootCmd(footend_interp);

            // Wait Key
            if (t == 0.0)
                waitKey("Press any key with Enter to execute.");

            // State recording
            RobotProfile profile;
            profile.time = ros::Time::now().toSec() - init_time_;
            profile.t = t;
            profile.pose = odom_interp;
            profile.foot_pos_list = state_traj.eval_foot_traj(sine_remap(t));
            profile.support_state = support_state;
            for (size_t k = 0; k < 6; ++k)
            {
                profile.foot_end_sdf[k] = gridmap_interface_->sdfValue(profile.foot_pos_list[k]);
            }
            robot_profile_.emplace_back(profile);

            // Update param t
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                state_sequence_planner_.dequeue_MCTsolution();
                
                // Switch tripod phase
                switchTripodPhase();

                if (state_sequence_planner_.get_state_traj_length() > 0)
                {
                    state_sequence_planner_.visClear();
                    ros::Duration(0.2).sleep();
                    state_traj = state_sequence_planner_.get_state_traj(0);
                    support_state = state_traj.eval_support_state(0.5);
                }
            }
            ros::spinOnce();
            rate_.sleep();
        } while (state_sequence_planner_.get_state_traj_length() > 0 && ros::ok());

        // Final position setting
        robot_interface_->setFootCmd(footend_interp);
    }

    double sine_remap(double t)
    {
        return 0.5 * (1 + std::sin(M_PI * (t - 0.5)));
    }

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
        }
    }
};