
rtk：

新建test_gps_nav_goal_rviz.cpp文件
cmakelists添加
```bash
    add_executable(test_gps_nav_goal_rviz test/test_gps_nav_goal_rviz.cpp)
    target_link_libraries(test_gps_nav_goal_rviz
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )
```
    新建test_gps_nav_goal_rviz.launch文件。
足端目标点可视化显示：  
在Hexapod201GaitPlanner.h文件中添加：
```bash
// Get current hexapod state for Raibert gait planning
      legged_traj_plan::hexapod_State current_state = getCurrentHexapodState();
      printHexapodState(current_state);
  

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
        footend_positions[leg_idx] = point_SE3Act(target_pose, world_foot_pos);

        // Apply keepConstBaseFootZ: set Z in base frame to constant value
        if (config_.keepConstBaseFootZ)
        {
          footend_positions[leg_idx][2] = config_.keepConstBaseFootZValue;
        }
        contact_states[leg_idx] = next_state.support_State_Now[leg_idx];
      }
       /* ===================== 插入 START ===================== */
      ROS_INFO_STREAM("========== Footend Positions Debug ==========");
      ROS_INFO_STREAM("Target Pose Translation: [" 
                      << target_pose.translation().transpose() << "]");
      ROS_INFO_STREAM("Target Pose RPY: [" 
                      << pinocchio::rpy::matrixToRpy(target_pose.rotation()).transpose() << "]");

      for (int leg_idx = 0; leg_idx < 6; leg_idx++)
      {
        ROS_INFO_STREAM("Leg[" << leg_idx << "] "
                      << (contact_states[leg_idx] ? "SUPPORT" : "SWING   ")
                      << " | Body: [" << footend_positions[leg_idx].transpose() << "]"
                      << " | Norm: " << footend_positions[leg_idx].norm());
      }

      {
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();
        for (int i = 0; i < 6; i++)
        {
          min_z = std::min(min_z, footend_positions[i][2]);
          max_z = std::max(max_z, footend_positions[i][2]);
        }
        ROS_INFO_STREAM("Foot Z range: min=" << min_z << ", max=" << max_z);
      }
      ROS_INFO_STREAM("=============================================");
      /* ===================== 插入 END ======================= */

      // 清除所有标记
    visualizer_.delAll();
    
    // 重新绘制路径
    std::vector<Eigen::Vector3d> path3d_current;
    for (const auto &pt : path2d)
    {
      path3d_current.emplace_back(pt[0], pt[1], pos3d[2]);
      visualizer_.visSphere(Point3D(pt[0], pt[1], pos3d[2]), 0.05);
    }
    visualizer_.visCurve(path3d_current);
    
    // 只显示三个摆动足的足端目标点
    for (int leg_idx = 0; leg_idx < 6; leg_idx++)
    {
      // 只可视化摆动腿（contact_states[leg_idx] == false）
      if (!contact_states[leg_idx])
      {
        // 将基座坐标系中的足端位置转换到世界坐标系进行可视化
        Eigen::Vector3d foot_world = target_pose.translation() + 
                                    target_pose.rotation() * footend_positions[leg_idx];
        
        // 在世界坐标系中可视化摆动足的足端目标点
        visualizer_.visSphere(Point3D(foot_world[0], 
                                      foot_world[1], 
                                      foot_world[2]), 0.10);
      }
    }

      // Use setStepCmd to set both body pose and foot positions
      if (robot_interface_type_ == "Hexapod201ROS")
        std::dynamic_pointer_cast<Hexapod201InterfaceROS>(robot_interface_)
            ->setStepCmd(target_pose, footend_positions, contact_states);
      else if (robot_interface_type_ == "Hexapod201Dummy")
        std::dynamic_pointer_cast<DummyHexapod201InterfaceROS>(robot_interface_)
            ->setStepCmd(target_pose, footend_positions, contact_states);
      else
        ROS_ERROR("Unsupported robot_interface_type for nav_callback.");
      plc_in_motion_ = true;
      
```
这里是ik求解可视化的步骤
头文件
```bash
// ==================== 1. 新增：引入 6 腿 IK 求解器 ====================
#include "ikfast_generated_RF.h"
#include "ikfast_generated_LF.h"
#include "ikfast_generated_RM.h"
#include "ikfast_generated_LM.h"
#include "ikfast_generated_RB.h"
#include "ikfast_generated_LB.h"
#include <iostream>
#include <array>
#include <Eigen/Dense>
```
在struct RobotProfile下面新增
```bash
struct LegParamsIK {
    double hip_x;
    double hip_y;
    double hip_z;
    double sign_haa;
    double sign_hfe;
    double sign_kfe;
};
```
在  // Status
  bool motion_lock_ = false;
  bool plc_in_motion_ = false;下面新增
```bash
  
    // Joint state publishing and Foot feedback subscribing
  ros::Subscriber foot_feedback_sub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher restart_trigger_pub_;
  std::vector<std::string> joint_names_ = {
      "LF_HAA", "LF_HFE", "LF_KFE",
      "LM_HAA", "LM_HFE", "LM_KFE",
      "LB_HAA", "LB_HFE", "LB_KFE",
      "RF_HAA", "RF_HFE", "RF_KFE",
      "RM_HAA", "RM_HFE", "RM_KFE",
      "RB_HAA", "RB_HFE", "RB_KFE"
      
  };
  // ==================== 【安全新增】IK 专用全局变量缓存 ====================
  std::array<LegParamsIK, 6> legs_params_ik_{};
  mutable std::array<Eigen::Vector3d, 6> last_q_all_legs_{};
  mutable std::array<bool, 6> leg_has_initialized_{};
  std::vector<Eigen::Vector3d> last_foot_pos_;

  // ==================== 【安全新增】不破坏原代码的独立 IK 过滤函数 ====================
  bool computeAllLegsIk(const std::vector<Eigen::Vector3d> &foot_targets, std::vector<double> &q_all_result) const 
{
    q_all_result.resize(18, 0.0);
    bool all_success = true;

    // 根据你 URDF 限制 HAA 限位在合理范围内，彻底掐断中间腿旋转 180 度缩进车体的可能
    double max_haa = 1.0;   // 约 57 度
    double min_haa = -1.0;  // 约 -57 度
    double max_hfe = 1.8;   double min_hfe = -1.8;
    double max_kfe = 2.5;   double min_kfe = -2.5;

    for (int i = 0; i < 6; ++i) {
        Eigen::Vector3d local_target = foot_targets[i]; // 基于 base_link 的目标点
        std::vector<Eigen::Vector3d> solutions;
        // 根据腿的索引调用对应的独立求解器命名空间
        switch (i) {
            case 0: solutions = ikfast_generated_LF::IKFast_trans3D(local_target); break;
            case 1: solutions = ikfast_generated_LM::IKFast_trans3D(local_target); break;
            case 2: solutions = ikfast_generated_LB::IKFast_trans3D(local_target); break;
            case 3: solutions = ikfast_generated_RF::IKFast_trans3D(local_target); break;
            case 4: solutions = ikfast_generated_RM::IKFast_trans3D(local_target); break;
            case 5: solutions = ikfast_generated_RB::IKFast_trans3D(local_target); break;
        }

        if (solutions.empty()) {
            all_success = false;
            // 发生物理奇异或超界断解时，保持上一帧角度
            if (leg_has_initialized_[i]) {
                q_all_result[i * 3 + 0] = last_q_all_legs_[i][0];
                q_all_result[i * 3 + 1] = last_q_all_legs_[i][1];
                q_all_result[i * 3 + 2] = last_q_all_legs_[i][2];
            }
            continue; 
        }

        Eigen::Vector3d best_sol;
        bool found_valid_sol = false;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto &sol : solutions) {
            double q0 = sol[0] * legs_params_ik_[i].sign_haa;
            double q1 = sol[1] * legs_params_ik_[i].sign_hfe;
            double q2 = sol[2] * legs_params_ik_[i].sign_kfe;

            // 关节物理限位过滤
            if (q0 < min_haa || q0 > max_haa || q1 < min_hfe || q1 > max_hfe || q2 < min_kfe || q2 > max_kfe) {
                continue; 
            }

            Eigen::Vector3d q_candidate(q0, q1, q2);
            if (leg_has_initialized_[i]) {
                double dist = (q_candidate - last_q_all_legs_[i]).squaredNorm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_sol = sol;
                    found_valid_sol = true;
                }
            } else {
                best_sol = sol;
                found_valid_sol = true;
                break; 
            }
        }

        if (!found_valid_sol) {
            if (leg_has_initialized_[i]) {
                q_all_result[i * 3 + 0] = last_q_all_legs_[i][0];
                q_all_result[i * 3 + 1] = last_q_all_legs_[i][1];
                q_all_result[i * 3 + 2] = last_q_all_legs_[i][2];
            }
            continue;
        }

        q_all_result[i * 3 + 0] = best_sol[0] * legs_params_ik_[i].sign_haa;
        q_all_result[i * 3 + 1] = best_sol[1] * legs_params_ik_[i].sign_hfe;
        q_all_result[i * 3 + 2] = best_sol[2] * legs_params_ik_[i].sign_kfe;

        last_q_all_legs_[i] = Eigen::Vector3d(q_all_result[i * 3 + 0], q_all_result[i * 3 + 1], q_all_result[i * 3 + 2]);
        leg_has_initialized_[i] = true;
    }
    return all_success;
}
```
在public:
  Hexapod201StateSequencePlanner()
      : ElSpiderAirPlannerBase(),
        state_sequence_planner_(swing_traj_planner_, gridmap_interface_,
                                robot_interface_),
        visualizer_(nh_, "world", "visualizer_markers"),
        rate_(100)
  {下面新增
```bash
    legs_params_ik_ = {{
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // LF
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // LM
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // LB
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // RF
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // RM
    {0.0,  0.1, 0.0,  1.0,  1.0,  1.0}, // RB
}};
    last_foot_pos_.assign(6, Eigen::Vector3d::Zero());
```
在cmd_sub_ = nh_.subscribe(
        "/cmd_vel", 1, &Hexapod201StateSequencePlanner::cmd_callback, this);下面新增
```bash
    restart_trigger_pub_ = nh_.advertise<std_msgs::Empty>("/trigger_system_restart", 1);
```
在plc_in_motion_sub_ =
        nh_.subscribe("/robot_is_moving", 1,
                      &Hexapod201StateSequencePlanner::plc_in_motion_callback, this);下面新增
```bash
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
```
```bash
void printHexapodState(const legged_traj_plan::hexapod_State& state)
  {
    std::cout << "===== Hexapod Current State =====" << std::endl;

    // Base pose
    std::cout << "Base Position: "
              << state.base_Pose_Now.position.x << ", "
              << state.base_Pose_Now.position.y << ", "
              << state.base_Pose_Now.position.z << std::endl;

    std::cout << "Base Orientation (RPY): "
              << state.base_Pose_Now.orientation.roll << ", "
              << state.base_Pose_Now.orientation.pitch << ", "
              << state.base_Pose_Now.orientation.yaw << std::endl;

    // Move direction
    std::cout << "Move Direction: "
              << state.move_Direction.x << ", "
              << state.move_Direction.y << ", "
              << state.move_Direction.z << std::endl;

    // Feet positions & support state
    for (int i = 0; i < 6; ++i)
    {
      std::cout << "Foot[" << i << "] Pos: "
                << state.feetPositionNow.foot[i].x << ", "
                << state.feetPositionNow.foot[i].y << ", "
                << state.feetPositionNow.foot[i].z
                << " | Support: " << state.support_State_Now[i]
                << " | Fault: " << state.faultLeg_State_Now[i]
                << std::endl;
    }

    std::cout << "===============================" << std::endl;
  }
  Eigen::Vector3d getFootPosInBaseMinusHipZ_Correct(
    const legged_traj_plan::FootState& foot_state,  // ⚠️ 传入原始的 foot_state
    int leg_id)
  {
    // ✅ 直接使用机身坐标系下的原始数据
    Eigen::Vector3d foot_in_base(
        foot_state.position[leg_id].x,
        foot_state.position[leg_id].y,
        foot_state.position[leg_id].z
    );

    // ✅ 减去髋关节 Z 偏移（URDF 中髋关节相对机身）
    foot_in_base.z() += 0.2055;

    return foot_in_base;
  }
```
```bash
plc_in_motion_ = true;
      
      // Wait for step completion
      // ros::Duration(step_duration).sleep();
      ros::Time step_start_time = ros::Time::now();
      while (ros::ok() && (plc_in_motion_ || (ros::Time::now() - step_start_time).toSec() < step_duration))
      {
        // 1. 实时获取机器人在运动过程中的当前真实足端位置
        legged_traj_plan::FootState rt_foot_state = robot_interface_->getFootStateFdb();
        
        // 2. 转换为带髋关节高度补偿的坐标
        std::vector<Eigen::Vector3d> rt_correct_foot(6);
        for (int i = 0; i < 6; ++i) {
          rt_correct_foot[i] = getFootPosInBaseMinusHipZ_Correct(rt_foot_state, i);
        }

        // 3. 实时计算 IK 并发布，让 RViz 产生连续动画
        std::vector<double> q_sol_all;
        if (computeAllLegsIk(rt_correct_foot, q_sol_all)) {
            sensor_msgs::JointState joint_msg;
            joint_msg.header.stamp = ros::Time::now();
            joint_msg.name = joint_names_;  
            joint_msg.position = q_sol_all;
            joint_state_pub_.publish(joint_msg); 
        } else {
            ROS_WARN_THROTTLE(2.0, "[旁路IK提示] 当前过渡轨迹靠近机械腿极限。");
        }

        // 以 50Hz 的频率循环更新 (0.02秒)
        ros::Duration(0.02).sleep();
        ros::spinOnce();
      }
    }
    
    if (current_waypoint >= path2d.size())
    {
      ROS_INFO("Navigation completed - reached final waypoint");
      
      // 1. 先解锁！（把收尾工作提前）
      motion_lock_ = false;
      gridmap_interface_->unlockMapUpdate();
      
      // 2. 再发布信号！
      ROS_WARN("Sending signal for FULL SYSTEM RESTART!");
      std_msgs::Empty trigger_msg;
      restart_trigger_pub_.publish(trigger_msg);
      
      return; // 直接返回，让 callback 完美结束
    }
    else
    {
      ROS_WARN("Navigation interrupted");
    }
    
    motion_lock_ = false;
    gridmap_interface_->unlockMapUpdate();
```

ik的添加
在/FLTPlanner/legged_traj_planner/legged_traj_plan_examples/include/legged_traj_plan_examples下添加文件夹ik
将六条腿的ik.h放在ik文件夹下面
在/Documents/201_planning_prj/src/FLTPlanner/legged_traj_planner/legged_traj_plan_examples/下添加lib文件
将六条腿的.a文件放在lib文件下
cmakelists修改
```bash
set(HEXAPOD_IK_6LEGS_LIBS
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_RF.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_LF.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_RM.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_LM.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_RB.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libikfast_generated_LB.a
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/legged_traj_plan_examples/ik
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(hexapod201_state_sequence_planner
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  # ==========================================
  # 【绝对安全新增】仅在这里把6条腿的库喂给你的节点
  # ==========================================
  ${HEXAPOD_IK_6LEGS_LIBS}
  # ==========================================
)
```