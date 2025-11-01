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
#include "Hexapod201GaitPlanner.h"
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
#include <ros_visualizer/ros_visualizer.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

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

  // Navigation parameters
  double maxStepLength;
  double maxYawChange;
  double navStepDuration;

  // Pose control parameters
  bool keepPoseHorizontal;
  bool keepConstBaseFootZ;
  double keepConstBaseFootZValue;

  void loadParams(ros::NodeHandle &nh, std::string ns = "StateSequencePlanner")
  {
    bool check_digit = true;
    check_digit &= nh.getParam(ns + "/rosRate", rosRate);
    check_digit &= nh.getParam(ns + "/stepTime", stepTime);
    check_digit &= nh.getParam(ns + "/execOnKeyboardCmd", execOnKeyboardCmd);
    check_digit &= nh.getParam(ns + "/tripodStepDuration", tripodStepDuration);
    check_digit &= nh.getParam(ns + "/tripodStanceDuration", tripodStanceDuration);
    check_digit &= nh.getParam(ns + "/maxStepLength", maxStepLength);
    check_digit &= nh.getParam(ns + "/maxYawChange", maxYawChange);
    check_digit &= nh.getParam(ns + "/navStepDuration", navStepDuration);
    // Load new pose control parameters
    check_digit &= nh.getParam(ns + "/keepPoseHorizontal", keepPoseHorizontal);
    check_digit &= nh.getParam(ns + "/keepConstBaseFootZ", keepConstBaseFootZ);
    check_digit &= nh.getParam(ns + "/keepConstBaseFootZValue", keepConstBaseFootZValue);
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
  Hexapod201GaitPlanner gait_planner_;

  // Cmd
  ros::Subscriber cmd_sub_;
  geometry_msgs::Twist cmd_;
  ros::Subscriber nav_goal_sub_;
  ros::Subscriber pose2d_sub_;
  ros::Subscriber plc_in_motion_sub_;

  // Interface
  StateSequencePlanner state_sequence_planner_;
  GridMapCmdVelExtrapolator gridmap_extrapolator_;

  // Status
  bool motion_lock_ = false;
  bool plc_in_motion_ = false;

  // Benchmarking
  double init_time_ = 0.0;
  std::vector<RobotProfile> robot_profile_;

  // Visualizer
  GCSVisualizer visualizer_;
  Hexapod2dNavRRT nav_rrt_planner_;

public:
  Hexapod201StateSequencePlanner()
      : ElSpiderAirPlannerBase(),
        state_sequence_planner_(swing_traj_planner_, gridmap_interface_,
                                robot_interface_),
        visualizer_(nh_, "world", "visualizer_markers"),
        rate_(100),
        gait_planner_(HexapodGaitType::GAIT_2, 0.4, 0.2)
  {
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
    plc_in_motion_sub_ =
        nh_.subscribe("/robot_is_moving", 1,
                      &Hexapod201StateSequencePlanner::plc_in_motion_callback, this);

    PosList pose_sample_pts;
    int len = 6;
    double delta = 0.2;
    for (int i = 0; i < len; i++)
    {
      for (int j = 0; j < len; j++)
      {
        pose_sample_pts.emplace_back(Eigen::Vector3d(
            i * delta - 0.5 * len * delta, j * delta - 0.5 * len * delta, 0));
      }
    }
    gridmap_extrapolator_.init(gridmap_interface_, pose_sample_pts, 0.405);
  }

  //=== Major Functions ===//
  // Simplified trajectory planner - only uses setBodyPoseCmd and setFootCmd
  void traj_planner()
  {
    double t = 0.0;
    double delta = 1 / config_.stepTime / config_.rosRate;

    MCTStateTransfer &state_traj = state_sequence_planner_.get_state_traj(0);
    pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(0.0);
    std::vector<Eigen::Vector3d> footend_interp =
        state_traj.eval_foot_traj(0.0);
    std::array<bool, 6> support_state = state_traj.eval_support_state(0.5);

    do
    {
      // Get Interpolated State
      odom_interp = state_traj.eval_torso_traj(sine_remap(t));
      support_state = state_traj.eval_support_state(sine_remap(t));

      // Footend position in world frame - simplified for Hexapod201
      footend_interp = state_traj.eval_foot_traj(sine_remap(t));

      for (size_t k = 0; k < 6; ++k)
      {
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
      for (size_t k = 0; k < 6; ++k)
      {
        profile.foot_end_sdf[k] =
            gridmap_interface_->sdfValue(profile.foot_pos_list[k]);
      }
      robot_profile_.emplace_back(profile);

      // Update param t
      t += delta;
      if (t > 1.0)
      {
        t = 0.0;
        state_sequence_planner_.dequeue_MCTsolution();

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
    // For python interface.
    if (robot_interface_type_ == "DummyHexapod201ROS")
      std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
          ->setStepBodyPoseCmd(odom_interp);
  }

  void run()
  {
    while (ros::ok())
    {
      ros::spinOnce();
    }
  }


  //=== Callbacks ===//
  void cmd_callback(const geometry_msgs::Twist &msg)
  {
    if (motion_lock_)
    {
      ROS_WARN("Robot is in motion, ignore new command.");
      return;
    }
    motion_lock_ = true;
    gridmap_interface_->lockMapUpdate();
    cmd_ = msg;
    state_sequence_planner_.visClear();

    // Fetch feedback
    ros::spinOnce();

    // Get current state and generate next state using gait planner
    legged_traj_plan::hexapod_State current_state = getCurrentHexapodState();
    legged_traj_plan::hexapod_State next_state = generateNextState(current_state, cmd_);

    bool ret = state_sequence_planner_.enqueue_MCTsolution(current_state, next_state);

    if (ret)
    {
      ROS_INFO("Hexapod201 gait planned successfully.");
    }
    else
    {
      ROS_WARN("Hexapod201 gait planning failed.");
    }

    // Visualization
    visualizer_.delAll();
    if (swing_traj_planner_config_.enableVis)
    {
      visualizer_.visSphere(Point3D(current_state.base_Pose_Now.position.x,
                                    current_state.base_Pose_Now.position.y,
                                    current_state.base_Pose_Now.position.z));
    }

    traj_planner();
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
  }

  void nav_callback(const geometry_msgs::PoseStamped &msg)
  {
    if (motion_lock_)
    {
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
    if (!nav_rrt_planner_.planPath(start, goal, gridmap_interface_, path2d))
    {
      ROS_WARN("2D RRT path planning failed.");
      motion_lock_ = false;
      gridmap_interface_->unlockMapUpdate();
      return;
    }
    // Visualize the path
    visualizer_.delAll();
    std::vector<Eigen::Vector3d> path3d;
    for (const auto &pt : path2d)
    {
      path3d.emplace_back(pt[0], pt[1], pos3d[2]); // Keep z from body pose
      visualizer_.visSphere(Point3D(pt[0], pt[1], pos3d[2]), 0.05);
    }
    visualizer_.visCurve(path3d);

    ROS_INFO_STREAM("2D RRT path found with " << path2d.size()
                                              << " waypoints.");
    // Parameters from config
    const double max_step_length = config_.maxStepLength;
    const double max_yaw_change = config_.maxYawChange;
    const double step_duration = config_.navStepDuration;

    // Traverse the path, interpolate if needed
    for (size_t i = 1; i < path2d.size(); i++)
    {
      Eigen::Vector2d prev = path2d[i - 1];
      Eigen::Vector2d curr = path2d[i];
      Eigen::Vector2d delta = curr - prev;
      double dist = delta.norm();
      int num_steps =
          std::max(1, static_cast<int>(std::ceil(dist / max_step_length)));
      for (int s = 1; s <= num_steps; ++s)
      {
        body_pose = robot_interface_->getBodyPoseFdb();
        double alpha = static_cast<double>(s) / num_steps;
        Eigen::Vector2d interp = prev + alpha * delta;

        // Create cmd_vel for this step in world frame
        geometry_msgs::Twist world_cmd_vel;
        world_cmd_vel.linear.x =
            delta[0] / num_steps / config_.tripodStepDuration;
        world_cmd_vel.linear.y =
            delta[1] / num_steps / config_.tripodStepDuration;
        world_cmd_vel.linear.z = 0.0;

        // Set orientation to face direction of movement
        double move_dir = acos(delta[0] / dist);
        if (delta[1] < 0)
        {
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
        if (delta_yaw > max_yaw_change)
        {
          delta_yaw = max_yaw_change;
        }
        if (delta_yaw < -max_yaw_change)
        {
          delta_yaw = -max_yaw_change;
        }
        rpy[2] += delta_yaw; // Update yaw
        if (rpy[2] > M_PI)
          rpy[2] -= 2 * M_PI;
        if (rpy[2] < -M_PI)
          rpy[2] += 2 * M_PI;
        target_pose.rotation() = pinocchio::rpy::rpyToMatrix(rpy);

        world_cmd_vel.angular.z = delta_yaw / config_.tripodStepDuration;

        // Transform world frame velocity to base frame velocity
        geometry_msgs::Twist step_cmd_vel;
        Eigen::Matrix3d world_to_base_rotation =
            body_pose.rotation().transpose();
        Eigen::Vector3d world_linear_vel(world_cmd_vel.linear.x,
                                         world_cmd_vel.linear.y,
                                         world_cmd_vel.linear.z);
        Eigen::Vector3d base_linear_vel =
            world_to_base_rotation * world_linear_vel;

        step_cmd_vel.linear.x = base_linear_vel[0];
        step_cmd_vel.linear.y = base_linear_vel[1];
        step_cmd_vel.linear.z = base_linear_vel[2];
        step_cmd_vel.angular.x = 0.0;
        step_cmd_vel.angular.y = 0.0;
        step_cmd_vel.angular.z =
            world_cmd_vel.angular.z; // Angular velocity same in both frames

        // Fit the ground
        gridmap_extrapolator_.update(target_pose, geometry_msgs::Twist{});
        target_pose = gridmap_extrapolator_.extrapolate(0.0);

        // Apply keepPoseHorizontal: set roll and pitch to 0
        if (config_.keepPoseHorizontal)
        {
          Eigen::Vector3d rpy_horizontal = pinocchio::rpy::matrixToRpy(target_pose.rotation());
          rpy_horizontal[0] = 0.0; // roll = 0
          rpy_horizontal[1] = 0.0; // pitch = 0
          // keep yaw unchanged: rpy_horizontal[2] remains the same
          target_pose.rotation() = pinocchio::rpy::rpyToMatrix(rpy_horizontal);
        }
        // Apply keepConstBaseFootZ: set base height to constant value
        if (config_.keepConstBaseFootZ)
        {
          target_pose.translation()[2] = 0.0 - config_.keepConstBaseFootZValue;
        }

        // Get current hexapod state for Raibert gait planning
        legged_traj_plan::hexapod_State current_state = getCurrentHexapodState();

        // Generate next state using gait planner
        legged_traj_plan::hexapod_State next_state = generateNextState(current_state, step_cmd_vel);

        // Convert foot positions from world frame to body frame for setStepCmd
        std::vector<Eigen::Vector3d> footend_positions(6);
        std::vector<bool> contact_states(6);
        for (int leg_idx = 0; leg_idx < 6; leg_idx++)
        {
          Eigen::Vector3d world_foot_pos(
              next_state.feetPositionNow.foot[leg_idx].x,
              next_state.feetPositionNow.foot[leg_idx].y,
              next_state.feetPositionNow.foot[leg_idx].z);
          // Transform from world frame to body frame
          footend_positions[leg_idx] =
              point_SE3Act(target_pose, world_foot_pos);

          // Apply keepConstBaseFootZ: set Z in base frame to constant value
          if (config_.keepConstBaseFootZ)
          {
            footend_positions[leg_idx][2] = config_.keepConstBaseFootZValue;
          }

          contact_states[leg_idx] = next_state.support_State_Now[leg_idx];
        }

        // Use setStepCmd to set both body pose and foot positions
        if (robot_interface_type_ == "Hexapod201ROS")
          std::dynamic_pointer_cast<Hexapod201InterfaceROS>(robot_interface_)
              ->setStepCmd(target_pose, footend_positions, contact_states);
        else if (robot_interface_type_ == "Hexapod201Dummy")
          std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(
              robot_interface_)
              ->setStepCmd(target_pose, footend_positions, contact_states);
        else
          ROS_ERROR("Unsupported robot_interface_type for nav_callback.");

        ros::Duration(step_duration).sleep(); // Step time, adjust as needed
        while (plc_in_motion_)
        {
          ros::Duration(0.1).sleep();
          ros::spinOnce();
        }
        ros::spinOnce();
      }
    }
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
  }

  void pose2d_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
  {
    if (motion_lock_)
    {
      ROS_WARN("Robot is in motion, ignore new nav goal.");
      return;
    }
    motion_lock_ = true;
    gridmap_interface_->lockMapUpdate();
    // Get current robot pose (x, y)
    V3d pos3d = get_cur_position();
    V2d start(pos3d[0], pos3d[1]);
    V2d goal(msg.pose.pose.position.x, msg.pose.pose.position.y);
    VEV2d path2d;
    // 进行二维的轨迹规划
    if (!nav_rrt_planner_.planPath(start, goal, gridmap_interface_, path2d))
    {
      ROS_WARN("2D RRT path planning failed.");
      motion_lock_ = false;
      gridmap_interface_->unlockMapUpdate();
      return;
    }
    // Visualize the path
    visualizer_.delAll();
    VEV3d path3d;
    // 令规划的路径的第三个维度一直都是当前的机身高度
    for (const auto &pt : path2d)
    {
      path3d.emplace_back(pt[0], pt[1], pos3d[2]); // Keep z from body pose
      visualizer_.visSphere(Point3D(pt[0], pt[1], pos3d[2]), 0.05);
    }
    visualizer_.visCurve(path3d);

    ROS_INFO_STREAM("2D RRT path found with " << path2d.size() << " points");
    path2d = interpolate_path(path2d, 0.30);
    ROS_INFO_STREAM("After interp, there is " << path2d.size() << " points");
    path3d.clear();
    for (const auto &pt : path2d)
    {
      path3d.emplace_back(pt[0], pt[1], pos3d[2]); // Keep z from body pose
    }
    ros_visualizer::VisStyle _style =
        ros_visualizer::VisStyle(0.0, 1, 0, 1, 0.05, 0.05, 0.05);
    visualizer_.visSphere(path3d, _style);
    nav_msgs::Path nav_path; // Declare and initialize nav_path
    for (int i = 0; i < path2d.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = path2d[i][0];
      pose.pose.position.y = path2d[i][1];
      pose.pose.position.z = 0.0;
      nav_path.poses.push_back(pose);
    }
    if (robot_interface_type_ == "Hexapod201ROS")
      std::dynamic_pointer_cast<Hexapod201InterfaceROS>(robot_interface_)
          ->setStepBodyPathCmd(nav_path);
    else if (robot_interface_type_ == "Hexapod201Dummy")
      std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
          ->setStepBodyPathCmd(nav_path);
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
  }

  void plc_in_motion_callback(const std_msgs::Bool::ConstPtr &msg)
  {
    plc_in_motion_ = msg->data;
  }

  using V2d = Eigen::Vector2d;
  using V3d = Eigen::Vector3d;
  using VEV2d = std::vector<V2d>;
  using VEV3d = std::vector<V3d>;

  //=== State Helpers ===//

  /**
   * @brief Get the cur yaw object
   * @return double
   */
  double get_cur_yaw()
  {
    pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
    return pinocchio::rpy::matrixToRpy(body_pose.rotation())[2];
  }

  /**
   * @brief Get the cur position object
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d get_cur_position()
  {
    pinocchio::SE3 body_pose = robot_interface_->getBodyPoseFdb();
    return body_pose.translation();
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

    // Get foot states and convert to world frame
    legged_traj_plan::FootState foot_state = robot_interface_->getFootStateFdb();
    for (int i = 0; i < 6; i++)
    {
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

  // Generate next state using the gait planner
  legged_traj_plan::hexapod_State
  generateNextState(const legged_traj_plan::hexapod_State &current_state,
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

    // Get contact pattern from gait planner
    std::array<bool, 6> contact_pattern = gait_planner_.getCurrentContactPattern();
    for (int i = 0; i < 6; i++)
    {
      next_state.support_State_Now[i] = contact_pattern[i];
    }

    // Update foot positions for swing legs using terrain-aware planning
    Eigen::Vector3d velocity(cmd_vel.linear.x, cmd_vel.linear.y, 0.0);
    for (int i = 0; i < 6; i++)
    {
      if (!contact_pattern[i]) // Swing leg
      {
        Eigen::Vector3d nominal_foothold = robot_interface_->getNominalFoothold(i);
        Eigen::Vector3d target_foothold = gait_planner_.computeTerrainAwareFoothold(
            next_pose, velocity, nominal_foothold, gridmap_interface_, i);

        // Set target foothold in world frame with terrain-aware height
        next_state.feetPositionNow.foot[i].x = target_foothold[0];
        next_state.feetPositionNow.foot[i].y = target_foothold[1];
        next_state.feetPositionNow.foot[i].z = target_foothold[2];
      }
    }

    // Advance gait phase for next step
    gait_planner_.advancePhase();

    return next_state;
  }


  //=== Utils Functions ===//

  double sine_remap(double t) { return 0.5 * (1 + std::sin(M_PI * (t - 0.5))); }

  /**
   * @brief 转换周期性变量val至(-bound, bound)
   * @param[in] bound         My Param doc
   * @param[in] val           My Param doc
   */
  void periodical_conv(double bound, double &val)
  {
    while (val < -abs(bound))
    {
      val += 2 * abs(bound);
    }
    while (val > abs(bound))
    {
      val -= 2 * abs(bound);
    }
    return;
  };

  /**
   * @brief 钳位val至(-bound, bound)
   * @param[in] bound         My Param doc
   * @param[in] val           My Param doc
   */
  void clamp(double bound, double &val)
  {
    if (val < -abs(bound))
    {
      val = -abs(bound);
    }
    if (val > abs(bound))
    {
      val = abs(bound);
    }
  };

  char waitKey(std::string info = "Press any key with Enter to continue.")
  {
    if (config_.execOnKeyboardCmd)
    {
      std::cout << info << std::endl;
      return getchar();
    }
    return 0;
  }

  /**
   * @brief 插值路径点，保证每两点之间的距离不超过max_step_length
   * @param[in] path2d            原始路径点
   * @param[in] max_step_length   最大步长
   * @return VEV2d                插值后的路径点
   */
  VEV2d interpolate_path(VEV2d &path2d, double max_step_length)
  {
    // Parameters
    VEV2d interpolated_path;
    for (int i = 1; i < path2d.size(); i++)
    {
      V2d last_point = path2d[i - 1];
      V2d cur_point = path2d[i];
      V2d pos_diff = cur_point - last_point;
      int split_num =
          std::max(1, (int)std::ceil(pos_diff.norm() / max_step_length));
      for (int i = 0; i < split_num; i++)
      {
        interpolated_path.push_back(last_point + pos_diff * i / split_num);
      }
      interpolated_path.push_back(cur_point);
    }
    return interpolated_path;
    // // 先确定aim_yaw和cur_yaw, 把自己的角度转换到这个线的正方向上
    // double aim_yaw = acos(pos_diff[0] / pos_diff.norm());
    // double cur_yaw = get_cur_yaw();
    // while (abs(aim_yaw - cur_yaw) > admit_yaw_err) {
    //   std::cout << "aim_yaw = " << aim_yaw << "cur_yaw = " << cur_yaw
    //             << "yaw_err = " << aim_yaw - cur_yaw << std::endl;
    //   // 进行相对位置控制
    //   // send_yaw()
    //   double yaw_control_command =
    //       abs(aim_yaw - cur_yaw) > max_yaw_change
    //           ? ((aim_yaw - cur_yaw) > 0 ? max_yaw_change :
    //           -max_yaw_change) : aim_yaw - cur_yaw;
    //   // 发送yaw轴控制指令,
    //   std::cout << "yaw_control_command = " << yaw_control_command
    //             << std::endl;
    //   ros::Duration(step_duration).sleep(); // Step time, adjust as needed
    // }
    // // 此时yaw轴角度已经和这段轨迹期望的yaw轴角度一致了
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
  }

};