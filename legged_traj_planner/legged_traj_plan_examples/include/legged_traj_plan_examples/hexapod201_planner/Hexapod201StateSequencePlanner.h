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
#include <algorithm>

/* internal project header files */
#include "../elspider_air_planner/ElSpiderAirPlannerBase.h"
#include "cout_color_config.h"
#include "legged_traj_plan/BodyState.h"
#include "legged_traj_plan/FootState.h"
#include "legged_traj_plan/hexapod_State.h"
#include "legged_traj_plan/whole_body_planner/CmdVelExtrapolator.h"
#include "legged_traj_plan/whole_body_planner/StateSequencePlanner.h"
#include <pinocchio/math/rpy.hpp>

/* external project header files */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "Hexapod2dNavRRT.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Simple tripod gait patterns for hexapod (legs 0-5: RF, RR, FL, LF, LR, RL)
enum class TripodPhase {
  PHASE_135, // Legs 1,3,5 (RR, LF, RL) swing, Legs 0,2,4 (RF, FL, LR) stance
  PHASE_246  // Legs 0,2,4 (RF, FL, LR) swing, Legs 1,3,5 (RR, LF, RL) stance
};

// Raibert heuristic for hexapod foothold placement
struct Hexapod201RaibertFootholdPlanner {
  double step_time;
  double stance_time;
  Eigen::Vector3d velocity_gain;

  Hexapod201RaibertFootholdPlanner(double step_t = 0.4, double stance_t = 0.2)
      : step_time(step_t), stance_time(stance_t), velocity_gain(0.5, 0.5, 0.0) {
  }

  Eigen::Vector3d computeFoothold(const pinocchio::SE3 &body_pose,
                                  const Eigen::Vector3d &body_velocity,
                                  const Eigen::Vector3d &nominal_foothold,
                                  int leg_index) {
    // Raibert heuristic: foothold = nominal + velocity_gain * body_velocity *
    // (step_time/2 + stance_time/2)
    double foothold_time = step_time / 2.0 + stance_time / 2.0;
    Eigen::Vector3d velocity_offset =
        velocity_gain.cwiseProduct(body_velocity) * foothold_time;

    // Transform nominal foothold to world frame
    Eigen::Vector3d world_nominal =
        point_SE3Act(body_pose.inverse(), nominal_foothold);

    // Add velocity-based offset for forward motion
    return world_nominal + velocity_offset;
  }
};

// For robot state recording
struct RobotProfile {
  double time;
  double t; // param time in traj
  pinocchio::SE3 pose;
  PosList foot_pos_list;
  std::array<double, 6> foot_end_sdf;
  std::array<bool, 6> support_state;
  geometry_msgs::Twist cmd_vel;
};

struct Hexapod201StateSequencePlannerConfig {
  int rosRate;
  double stepTime;

  bool execOnKeyboardCmd;

  // Tripod gait parameters
  double tripodStepDuration;
  double tripodStanceDuration;

  void loadParams(ros::NodeHandle &nh,
                  std::string ns = "StateSequencePlanner") {
    bool check_digit = true;
    check_digit &= nh.getParam(ns + "/rosRate", rosRate);
    check_digit &= nh.getParam(ns + "/stepTime", stepTime);
    check_digit &= nh.getParam(ns + "/execOnKeyboardCmd", execOnKeyboardCmd);
    check_digit &= nh.getParam(ns + "/tripodStepDuration", tripodStepDuration);
    check_digit &=
        nh.getParam(ns + "/tripodStanceDuration", tripodStanceDuration);
    if (!check_digit) {
      ROS_ERROR("Failed to load Hexapod201StateSequencePlannerConfig.");
    }
  }
};

class Hexapod201StateSequencePlanner : public ElSpiderAirPlannerBase {
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

  ros::Subscriber nav_goal_sub_;
  ros::Subscriber pose2d_sub_;
  Hexapod2dNavRRT nav_rrt_planner_;

public:
  Hexapod201StateSequencePlanner()
      : ElSpiderAirPlannerBase(),
        state_sequence_planner_(swing_traj_planner_, gridmap_interface_,
                                robot_interface_),
        visualizer_(nh_, "odom", "visualizer_markers"), rate_(100),
        current_tripod_phase_(TripodPhase::PHASE_135),
        hexapod_raibert_planner_(0.4, 0.2) {
    config_.loadParams(nh_);
    rate_ = ros::Rate(config_.rosRate);
    init_time_ = ros::Time::now().toSec();
    cmd_sub_ = nh_.subscribe(
        "/cmd_vel", 1, &Hexapod201StateSequencePlanner::cmd_callback, this);
    nav_goal_sub_ =
        nh_.subscribe("/move_base_simple/goal", 1,
                      &Hexapod201StateSequencePlanner::nav_callback, this);
    pose2d_sub_ =
        nh_.subscribe("/initialpose", 1,
                      &Hexapod201StateSequencePlanner::pose2d_callback, this);

    PosList pose_sample_pts;
    int len = 6;
    double delta = 0.2;
    for (int i = 0; i < len; i++) {
      for (int j = 0; j < len; j++) {
        pose_sample_pts.emplace_back(Eigen::Vector3d(
            i * delta - 0.5 * len * delta, j * delta - 0.5 * len * delta, 0));
      }
    }
    gridmap_extrapolator_.init(gridmap_interface_, pose_sample_pts);
  }

  // cmd_vel callback
  void cmd_callback(const geometry_msgs::Twist &msg) {
    if (motion_lock_)
      ROS_WARN("Robot is in motion, ignore new command.");
    else {
      motion_lock_ = true;
      gridmap_interface_->lockMapUpdate();
      cmd_ = msg;
      state_sequence_planner_.visClear();

      // Fetch feedback
      ros::spinOnce();

      // Use simple tripod gait planner (Raibert-style)
      legged_traj_plan::hexapod_State current_state = getCurrentHexapodState();
      legged_traj_plan::hexapod_State next_state =
          generateNextTripodState(current_state, cmd_);

      bool ret = state_sequence_planner_.enqueue_MCTsolution(current_state,
                                                             next_state);

      if (ret) {
        ROS_INFO("Hexapod201 tripod gait planned successfully.");
      } else {
        ROS_WARN("Hexapod201 tripod gait planning failed.");
      }

      // Visualization
      visualizer_.delAll();

      if (swing_traj_planner_config_.enableVis) {
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
  void nav_callback(const geometry_msgs::PoseStamped &msg) {
    if (motion_lock_) {
      ROS_WARN("Robot is in motion, ignore new nav goal.");
      return;
    }
    motion_lock_ = true;
    gridmap_interface_->lockMapUpdate();
    // Get current robot pose (x, y)
    pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
    Eigen::Vector3d pos3d = body_pose.translation();
    Eigen::Vector2d start(pos3d[0], pos3d[1]);
    Eigen::Vector2d goal(msg.pose.position.x, msg.pose.position.y);
    std::vector<Eigen::Vector2d> path2d;
    if (!nav_rrt_planner_.planPath(start, goal, gridmap_interface_, path2d)) {
      ROS_WARN("2D RRT path planning failed.");
      motion_lock_ = false;
      gridmap_interface_->unlockMapUpdate();
      return;
    }
    // Visualize the path
    visualizer_.delAll();
    std::vector<Eigen::Vector3d> path3d;
    for (const auto &pt : path2d) {
      path3d.emplace_back(pt[0], pt[1], pos3d[2]); // Keep z from body pose
      visualizer_.visSphere(Point3D(pt[0], pt[1], pos3d[2]), 0.05);
    }
    visualizer_.visCurve(path3d);

    ROS_INFO_STREAM("2D RRT path found with " << path2d.size()
                                              << " waypoints.");
    // Parameters
    const double max_step_length = 0.3;     // meters
    const double max_yaw_change = M_PI / 4; // radians
    const double step_duration = 2.0;       // seconds

    // Traverse the path, interpolate if needed
    for (size_t i = 1; i < path2d.size(); i++) {
      Eigen::Vector2d prev = path2d[i - 1];
      Eigen::Vector2d curr = path2d[i];
      Eigen::Vector2d delta = curr - prev;
      double dist = delta.norm();
      int num_steps =
          std::max(1, static_cast<int>(std::ceil(dist / max_step_length)));
      for (int s = 1; s <= num_steps; ++s) {
        body_pose = robot_interface_->getBodyPoseFdb();
        double alpha = static_cast<double>(s) / num_steps;
        Eigen::Vector2d interp = prev + alpha * delta;
        // Set orientation to face direction of movement
        double move_dir = acos(delta[0] / dist);
        if (delta[1] < 0) {
          move_dir = -move_dir; // Adjust for quadrant
        }
        pinocchio::SE3 target_pose = body_pose;
        target_pose.translation()[0] = interp[0];
        target_pose.translation()[1] = interp[1];
        // Set move_dir in target_pose (keep roll, pitch from body_pose)
        Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(body_pose.rotation());
        double delta_yaw = move_dir - rpy[2];
        if (delta_yaw > M_PI)
          delta_yaw -= 2 * M_PI;
        if (delta_yaw < -M_PI)
          delta_yaw += 2 * M_PI;
        if (delta_yaw > max_yaw_change) {
          delta_yaw = max_yaw_change;
        }
        if (delta_yaw < -max_yaw_change) {
          delta_yaw = -max_yaw_change;
        }
        rpy[2] += delta_yaw; // Update yaw
        if (rpy[2] > M_PI)
          rpy[2] -= 2 * M_PI;
        if (rpy[2] < -M_PI)
          rpy[2] += 2 * M_PI;
        target_pose.rotation() = pinocchio::rpy::rpyToMatrix(rpy);
        // Fit the ground
        gridmap_extrapolator_.update(target_pose, geometry_msgs::Twist{});
        target_pose = gridmap_extrapolator_.extrapolate(0.0);
        std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
            ->setStepBodyPoseCmd(target_pose);
        ros::Duration(step_duration).sleep(); // Step time, adjust as needed
      }
    }
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
  }

  void periodical_conv(double bound, double &val) {
    while (val < -abs(bound)) {
      val += 2 * abs(bound);
    }
    while (val > abs(bound)) {
      val -= 2 * abs(bound);
    }
    return;
  };
  void clamp(double bound, double &val) {
    if (val < -abs(bound)) {
      val = -abs(bound);
    }
    if (val > abs(bound)) {
      val = abs(bound);
    }
    return;
  };
  double get_cur_yaw() {
    pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
    return pinocchio::rpy::matrixToRpy(body_pose.rotation())[2];
  }
  // 传入一个稀疏的path_2d，按照其中指定的参数插值成需要到达的点
  std::vector<Eigen::Vector2d>
  interplodate_path(std::vector<Eigen::Vector2d> &path2d,
                    pinocchio::SE3 &body_pose) {
    // Parameters
    const double max_step_length = 0.3;      // meters
    const double max_yaw_change = M_PI / 36; // radians 一次转动约5度yaw角度
    const double step_duration = 1.0;        // seconds
    const double admit_yaw_err = 3.0 / 180.0 * M_PI; // seconds

    for (int i = 1; i < path2d.size(); i++) {
      Eigen::Vector2d last_point = path2d[i - 1];
      Eigen::Vector2d aim_point = path2d[i];
      Eigen::Vector2d pos_diff = aim_point - last_point;
      // 先确定aim_yaw和cur_yaw, 把自己的角度转换到这个线的正方向上
      double aim_yaw = acos(pos_diff[0] / pos_diff.norm());
      double cur_yaw = get_cur_yaw();
      while (abs(aim_yaw - cur_yaw) > admit_yaw_err) {
        std::cout << "aim_yaw = " << aim_yaw << "cur_yaw = " << cur_yaw
                  << "yaw_err = " << aim_yaw - cur_yaw << std::endl;
        // 进行相对位置控制
        // send_yaw()
        double yaw_control_command =
            abs(aim_yaw - cur_yaw) > max_yaw_change
                ? ((aim_yaw - cur_yaw) > 0 ? max_yaw_change : -max_yaw_change)
                : aim_yaw - cur_yaw;
        // 发送yaw轴控制指令,
        std::cout << "yaw_control_command = " << yaw_control_command
                  << std::endl;
        ros::Duration(step_duration).sleep(); // Step time, adjust as needed
      }
      // 此时yaw轴角度已经和这段轨迹期望的yaw轴角度一致了
    }
    // // Traverse the path, interpolate if needed
    // // 这里的插值保证先有转动，再进行X方向的前进移动，分开独立进行
    // // 分别打印插值之前和之后的结果来RVIZ中进行显示
    // for (size_t i = 1; i < path2d.size(); i++) {
    //   Eigen::Vector2d prev = path2d[i - 1];
    //   Eigen::Vector2d curr = path2d[i];
    //   Eigen::Vector2d delta = curr - prev;
    //   double dist = delta.norm();
    //   // std::ceil将返回大于或等于该数的最小整数
    //   // 计算一下这个OMPL搜索出的路径的上一个点和这个点需要走几步
    //   // FIXME: 这里可能缺少了一个转动的部分
    //   int num_steps =
    //       std::max(1, static_cast<int>(std::ceil(dist / max_step_length)));
    //   for (int s = 1; s <= num_steps; s++) {
    //     body_pose = robot_interface_->getBodyPoseFdb();
    //     double alpha = (double)s / num_steps;
    //     Eigen::Vector2d interp = prev + alpha * delta;
    //     // Set orientation to face direction of movement
    //     double move_dir = acos(delta[0] / dist);
    //     if (delta[1] < 0) {
    //       move_dir = -move_dir; // Adjust for quadrant
    //     }
    //     pinocchio::SE3 target_pose = body_pose;
    //     target_pose.translation()[0] = interp[0];
    //     target_pose.translation()[1] = interp[1];
    //     // Set move_dir in target_pose (keep roll, pitch from body_pose)
    //     Eigen::Vector3d rpy =
    //     pinocchio::rpy::matrixToRpy(body_pose.rotation()); double delta_yaw =
    //     move_dir - rpy[2];
    //     // clamp val to -bound to bound

    //     periodical_conv(M_PI, delta_yaw);
    //     clamp(max_yaw_change, delta_yaw);
    //     rpy[2] += delta_yaw; // Update yaw
    //     periodical_conv(M_PI, rpy[2]);
    //     target_pose.rotation() = pinocchio::rpy::rpyToMatrix(rpy);
    //     // Fit the ground
    //     gridmap_extrapolator_.update(target_pose, geometry_msgs::Twist{});
    //     target_pose = gridmap_extrapolator_.extrapolate(0.0);
    //     std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
    //         ->setStepBodyPoseCmd(target_pose);
    //     ros::Duration(step_duration).sleep(); // Step time, adjust as needed
    //   }
    // }
    return;
  }
  void pose2d_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    if (motion_lock_) {
      ROS_WARN("Robot is in motion, ignore new nav goal.");
      return;
    }
    motion_lock_ = true;
    gridmap_interface_->lockMapUpdate();
    // Get current robot pose (x, y)
    pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
    Eigen::Vector3d pos3d = body_pose.translation();
    Eigen::Vector2d start(pos3d[0], pos3d[1]);
    Eigen::Vector2d goal(msg.pose.pose.position.x, msg.pose.pose.position.y);
    std::cout << GREEN << "get pose2d goal:"
              << " cur_pose x: " << pos3d[0] << " cur_pose y: " << pos3d[1]
              << " cur_pose z: " << pos3d[2] << " aim_pose x: " << goal[0]
              << " aim_pose y: " << goal[1] << RESET << std::endl;
    std::vector<Eigen::Vector2d> path2d;
    // 进行二维的轨迹规划
    if (!nav_rrt_planner_.planPath(start, goal, gridmap_interface_, path2d)) {
      ROS_WARN("2D RRT path planning failed.");
      motion_lock_ = false;
      gridmap_interface_->unlockMapUpdate();
      return;
    }
    // Visualize the path
    visualizer_.delAll();
    std::vector<Eigen::Vector3d> path3d;
    // 令规划的路径的第三个维度一直都是当前的机身高度
    for (const auto &pt : path2d) {
      path3d.emplace_back(pt[0], pt[1], pos3d[2]); // Keep z from body pose
      visualizer_.visSphere(Point3D(pt[0], pt[1], pos3d[2]), 0.05);
    }
    visualizer_.visCurve(path3d);

    ROS_INFO_STREAM("2D RRT path found with " << path2d.size()
                                              << " waypoints.");
    interplodate_path(path2d, body_pose);
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
  }
  // Get current robot state and convert to hexapod_State
  legged_traj_plan::hexapod_State getCurrentHexapodState() {
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
    legged_traj_plan::FootState foot_state =
        robot_interface_->getFootStateFdb();
    for (int i = 0; i < 6; i++) {
      Point3D base_foot_pos(foot_state.position[i].x, foot_state.position[i].y,
                            foot_state.position[i].z);
      Point3D world_foot_pos = point_SE3Act(body_pose.inverse(), base_foot_pos);
      hexapodState.feetPositionNow.foot[i].x = world_foot_pos[0];
      hexapodState.feetPositionNow.foot[i].y = world_foot_pos[1];
      hexapodState.feetPositionNow.foot[i].z = world_foot_pos[2];
      hexapodState.support_State_Now[i] = true; // Default all stance
      hexapodState.faultLeg_State_Now[i] = 0;   // Normal
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
  legged_traj_plan::hexapod_State
  generateNextTripodState(const legged_traj_plan::hexapod_State &current_state,
                          const geometry_msgs::Twist &cmd_vel) {
    legged_traj_plan::hexapod_State next_state = current_state;

    // Get current pose and update the extrapolator
    pinocchio::SE3 current_pose = XYZRPY2SE3(current_state.base_Pose_Now);
    gridmap_extrapolator_.update(current_pose, cmd_vel);

    // Use terrain-aware pose extrapolation
    double dt = config_.tripodStepDuration;
    pinocchio::SE3 next_pose = gridmap_extrapolator_.extrapolate(dt);

    next_state.base_Pose_Now = SE32XYZRPY(next_pose);

    // Set contact pattern based on current tripod phase
    std::array<bool, 6> contact_pattern =
        getTripodContactPattern(current_tripod_phase_);
    for (int i = 0; i < 6; i++) {
      next_state.support_State_Now[i] = contact_pattern[i];
    }

    // Update foot positions using Raibert heuristic for swing legs
    Eigen::Vector3d velocity(cmd_vel.linear.x, cmd_vel.linear.y, 0.0);
    for (int i = 0; i < 6; i++) {
      if (!contact_pattern[i]) // Swing leg
      {
        Eigen::Vector3d nominal_foothold =
            robot_interface_->getNominalFoothold(i);
        Eigen::Vector3d target_foothold =
            hexapod_raibert_planner_.computeFoothold(next_pose, velocity,
                                                     nominal_foothold, i);

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
  std::array<bool, 6> getTripodContactPattern(TripodPhase phase) {
    std::array<bool, 6> pattern;
    if (phase == TripodPhase::PHASE_135) {
      pattern[0] = true;
      pattern[1] = false;
      pattern[2] = true;
      pattern[3] = false;
      pattern[4] = true;
      pattern[5] = false;
    } else // PHASE_246
    {
      pattern[0] = false;
      pattern[1] = true;
      pattern[2] = false;
      pattern[3] = true;
      pattern[4] = false;
      pattern[5] = true;
    }
    return pattern;
  }

  void switchTripodPhase() {
    current_tripod_phase_ = (current_tripod_phase_ == TripodPhase::PHASE_135)
                                ? TripodPhase::PHASE_246
                                : TripodPhase::PHASE_135;
  }

  // Simplified trajectory planner - only uses setBodyPoseCmd and setFootCmd
  void traj_planner() {
    double t = 0.0;
    double delta = 1 / config_.stepTime / config_.rosRate;

    MCTStateTransfer &state_traj = state_sequence_planner_.get_state_traj(0);
    pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
    std::vector<Eigen::Vector3d> footend_interp =
        state_traj.eval_foot_traj(0.0);
    std::array<bool, 6> support_state = state_traj.eval_support_state(0.5);

    do {
      // Get Interpolated State
      odom_interp = state_traj.eval_torso_traj(sine_remap(t));
      support_state = state_traj.eval_support_state(sine_remap(t));

      // Footend position in world frame - simplified for Hexapod201
      footend_interp = state_traj.eval_foot_traj(sine_remap(t));

      for (size_t k = 0; k < 6; ++k) {
        // Convert to BASE
        footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
      }

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
      for (size_t k = 0; k < 6; ++k) {
        profile.foot_end_sdf[k] =
            gridmap_interface_->sdfValue(profile.foot_pos_list[k]);
      }
      robot_profile_.emplace_back(profile);

      // Update param t
      t += delta;
      if (t > 1.0) {
        t = 0.0;
        state_sequence_planner_.dequeue_MCTsolution();

        // Switch tripod phase
        switchTripodPhase();

        if (state_sequence_planner_.get_state_traj_length() > 0) {
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
    // For python interface.
    std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
        ->setStepBodyPoseCmd(odom_interp);
  }

  double sine_remap(double t) { return 0.5 * (1 + std::sin(M_PI * (t - 0.5))); }

  char waitKey(std::string info = "Press any key with Enter to continue.") {
    if (config_.execOnKeyboardCmd) {
      std::cout << info << std::endl;
      return getchar();
    }
    return 0;
  }

  void run() {
    while (ros::ok()) {
      ros::spinOnce();
    }
  }
};