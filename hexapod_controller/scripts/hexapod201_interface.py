#!/usr/bin/env python3

from deprecated import deprecated
import rospy
import numpy as np

from abc import ABC
import time
import threading
import math
from typing import Dict, Any, Optional, Tuple, List
from enum import IntEnum
from beifu_control_vals import *
from hexapod201_base_interface import *
from geometry_msgs.msg import Twist, PoseStamped, Pose
import geometry_msgs.msg
from std_msgs.msg import Header, UInt64, Bool      # 导入 UInt64 和 Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path
import tf2_ros
import copy
# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle  # pyright: ignore[reportAttributeAccessIssue]
# Import FootState message
from legged_traj_plan.msg import FootState
from std_msgs.msg import Bool

FOOT_REMAP = [3, 4, 5, 0, 1, 2]  # Remap from FootElAir to Foot201


class Hexapod201Interface(Hexapod201BaseInterface):
    """Real hexapod interface using PLC communication"""

    def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
        super().__init__(node_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.world_frame = "world"
        self.base_frame = "base_link"

        self.task_status_pub = rospy.Publisher('/robot/task_finished', Bool, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber('/Odometry', Odometry, self._robot_pose_sub_callback)
        self.inc_cmd_sub = rospy.Subscriber('/hexapod/incremental_cmd', Twist, self.incremental_cmd_callback, queue_size=1)
        rospy.loginfo("Incremental cmd subscriber registered")
        self.joint_encoder_timer = rospy.Timer(rospy.Duration(0, int(2e7)), self.joint_encoder_timer_callback)

        # ★ 新增：PLC 状态发布，与通信桥话题对齐
        self.plc_state_pub = rospy.Publisher('/hexapod/plc_state', UInt64, queue_size=1)
        # ★ 新增：使能控制订阅（来自 wq_unified_bridge）
        rospy.Subscriber('/hexapod/enable', Bool, self._enable_callback)

        self.plc_ip = plc_ip
        self.plc = None
        self.plc_connected = False
        self.cpp = None
        self.cpp_connected = False

        # PLC symbols
        self.symbol_Cmd_Time = None
        self.symbol_Cmd_Gait = None
        self.symbol_Cmd_Pose = None
        self.symbol_CtrlCmd = None
        self.symbol_State = None
        self.symbol_QState = None
        self.symbol_PTActPos = None

        # CPP symbols for free gait
        self.symbol_ReqPTCmd = None
        self.symbol_ReqFlag = None
        self.ReqPTCmd = None

        # Movement parameters
        self.cmdTime = None
        self.cmdGait = None
        self.cmdPose = None

        # 判断一个位置到了没有的容许误差
        self.admit_pose_limit = 0.10  # 0.10m
        self.admit_yaw_limit = 4.0 / 180.0 * math.pi  # 4° degree

        # StateSequencePlanner params
        self.keepPoseHorizontal = rospy.get_param("~StateSequencePlanner/keepPoseHorizontal", True)
        self.keepConstBaseFootZ = rospy.get_param("~StateSequencePlanner/keepConstBaseFootZ", True)
        self.keepConstBaseFootZValue = rospy.get_param("~StateSequencePlanner/keepConstBaseFootZValue", -0.45)

        # Connect to PLC and CPP
        self._connect_plc()

        rospy.loginfo("Real hexapod interface initialized")

    def cleanup(self):
        """Cleanup PLC and CPP connections"""
        if self.plc_connected and self.plc:
            try:
                self.symbol_State.write(State.DISENABLE)  # type: ignore
                self.plc.close()
                rospy.loginfo("PLC connection closed")
            except Exception as e:
                rospy.logerr(f"Error closing PLC connection: {str(e)}")

        if self.cpp_connected and self.cpp:
            try:
                self.cpp.close()
                rospy.loginfo("CPP connection closed")
            except Exception as e:
                rospy.logerr(f"Error closing CPP connection: {str(e)}")

    # PLC related methods
    def _connect_plc(self):
        """Establish PLC and CPP connections"""

        # PLC connection
        self.plc = pyads.Connection(self.plc_ip, pyads.PORT_TC3PLC1, '192.168.3.101')
        self.plc.open()

        # CPP connection for free gait
        self.cpp = pyads.Connection(self.plc_ip, 351, '192.168.3.101')
        self.cpp.open()

        # Initialize PLC symbols
        self.symbol_Cmd_Time = self.plc.get_symbol('MAIN.PTCmd.TM', structure_def=stTime_def)
        self.symbol_Cmd_Gait = self.plc.get_symbol('MAIN.PTCmd.Gait', structure_def=stGait_def)
        self.symbol_Cmd_Pose = self.plc.get_symbol('MAIN.PTCmd.Pose', structure_def=stPose_def)
        self.symbol_CtrlCmd = self.plc.get_symbol('MAIN.CtrlCmd', plc_datatype="UDINT")
        self.symbol_CtrlCmd.symbol_type = pyads.PLCTYPE_UDINT
        self.symbol_CtrlCmd.plc_type = pyads.PLCTYPE_UDINT
        self.symbol_State = self.plc.get_symbol('MAIN.state', plc_datatype="UDINT")
        self.symbol_State.symbol_type = pyads.PLCTYPE_UDINT
        self.symbol_State.plc_type = pyads.PLCTYPE_UDINT
        self.symbol_QState = self.plc.get_symbol('MAIN.Q_State')
        self.symbol_QState.symbol_type = pyads.PLCTYPE_UDINT
        self.symbol_QState.plc_type = pyads.PLCTYPE_UDINT
        self.symbol_PTActPos = self.plc.get_symbol('MAIN.PTActPos', structure_def=stPose_def)
        self.symbol_PTCmdPos = self.plc.get_symbol('MAIN.PTCmdPos', structure_def=stPose_def)
        self.symbol_QJointPos = self.plc.get_symbol('MAIN.Q_JointPos', structure_def=stJointPos_def)
        self.symbol_QPTVel_Force = self.plc.get_symbol('MAIN.Q_PTVelForce', structure_def=stPose_def)
        self.symbol_QJointVel = self.plc.get_symbol('MAIN.Q_JointVel', structure_def=stJointPos_def)

        # Initialize CPP symbols for free gait
        self.symbol_ReqPTCmd = self.cpp.get_symbol('CPP.Inputs.ReqPTCmd', structure_def=stPose_def)
        self.symbol_ReqFlag = self.cpp.get_symbol('CPP.Inputs.ReqFlag')

        # Enable auto-update for feedback
        self.symbol_QState.auto_update = True
        self.symbol_ReqFlag.auto_update = True
        self.symbol_PTActPos.auto_update = True
        self.symbol_PTCmdPos.auto_update = True
        self.symbol_QJointPos.auto_update = True
        self.symbol_QPTVel_Force.auto_update = True
        self.symbol_QJointVel.auto_update = True

        self.plc_connected = True
        self.cpp_connected = True
        rospy.loginfo("PLC and CPP connections established")

    def _is_plc_enabled(self):
        return self.symbol_QState.value == State.FEEDMOV

    def _enable_plc(self):
        """Enable PLC for movement"""
        if not self.plc_connected:
            return False
        if self._is_plc_enabled():
            return True
        try:
            print("start self.symbol_State.write(State.ENABLE)")
            self.symbol_State.write(State.ENABLE)  # pyright: ignore[reportOptionalMemberAccess]
            print("finish self.symbol_State.write(State.ENABLE)")
            timeout = 10.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                if self.symbol_QState.value == State.FEEDMOV:  # type: ignore
                    rospy.loginfo("PLC enabled successfully")
                    return True
                rospy.sleep(0.1)
            rospy.logwarn("PLC enable timeout")
            return False
        except Exception as e:
            rospy.logerr(f"PLC enable failed: {str(e)}")
            return False

    # ★ 新增：使能回调
    def _enable_callback(self, msg: Bool):
        if msg.data:
            rospy.loginfo("Received enable command from wq_unified_bridge, enabling PLC...")
            self._enable_plc()

    def _read_plc_parameters(self):
        """Read current PLC parameters"""
        if not self.plc_connected:
            return False
        if self.cmdTime is None:
            self.cmdTime = self.symbol_Cmd_Time.read()  # type: ignore
        if self.cmdGait is None:
            self.cmdGait = self.symbol_Cmd_Gait.read()  # type: ignore
        if self.cmdPose is None:
            self.cmdPose = self.symbol_Cmd_Pose.read()  # type: ignore
        return True

    # Callbacks
    def _robot_pose_sub_callback(self, msg: Odometry):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                msg.header.stamp,
                rospy.Duration(0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as ex:
            rospy.logwarn_throttle(5.0, f"TF lookup failed: {ex}")
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.current_pose.position.x = translation.x
        self.current_pose.position.y = translation.y
        self.current_pose.position.z = translation.z
        self.current_pose.orientation = rotation
        return

    # Timer callbacks
    def update_feedback(self, event):
        """Periodic update of feedback from PLC"""
        super().update_feedback(event)

        if not self.plc_connected:
            return

        # ★ 无论使能与否，都发布当前 PLC 状态，供通信桥使用
        try:
            qstate = self.symbol_QState.value
            self.plc_state_pub.publish(UInt64(data=qstate))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Failed to publish PLC state: {e}")

        if not self._is_plc_enabled():
            return
        # Update foot positions
        self._update_footpos_from_plc()

    @deprecated("PLC pose is not correct, update from odom instead.")
    def _update_current_pose_from_plc(self):
        """Update current pose from PLC feedback"""
        if not self.plc_connected:
            return
        try:
            act_pos = self.symbol_PTActPos.read()  # type: ignore
            if act_pos:
                self.current_pose.position.x = act_pos["X"]
                self.current_pose.position.y = act_pos["Y"]
                self.current_pose.position.z = act_pos["Z"]
                quat = quaternion_from_euler(act_pos["Roll"], act_pos["Pitch"], act_pos["Yaw"])
                self.current_pose.orientation.w = quat[0]
                self.current_pose.orientation.x = quat[1]
                self.current_pose.orientation.y = quat[2]
                self.current_pose.orientation.z = quat[3]
        except Exception as e:
            rospy.logwarn(f"Failed to read current pose from PLC: {str(e)}")

    def _update_footpos_from_plc(self):
        """Update foot positions from PLC feedback"""
        if not self.plc_connected or not self._is_plc_enabled():
            return
        act_pos = self.symbol_PTActPos.read()
        if act_pos:
            self.foot_positions = np.array([
                [act_pos['X1'], act_pos['Y1'], act_pos['Z1']],
                [act_pos['X2'], act_pos['Y2'], act_pos['Z2']],
                [act_pos['X3'], act_pos['Y3'], act_pos['Z3']],
                [act_pos['X4'], act_pos['Y4'], act_pos['Z4']],
                [act_pos['X5'], act_pos['Y5'], act_pos['Z5']],
                [act_pos['X6'], act_pos['Y6'], act_pos['Z6']],
            ])

    # Motion Interface
    # Simple movement methods
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Move hexapod to target pose using PLC"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return False
        try:
            if not self._enable_plc():
                return False
            if not self._read_plc_parameters():
                return False
            self.cmdTime["TA"] = 1.5
            self.cmdTime["TM"] = 1.5
            self.cmdTime["TD"] = 0.0
            self.cmdTime["TZ"] = 0.2
            self.symbol_Cmd_Time.write(self.cmdTime)
            self.cmdGait["GaitMode"] = 1
            self.cmdGait["GaitDF"] = 0.5
            self.cmdGait["SwapHigh"] = 100.0
            self.cmdGait["LegNum"] = 0
            self.cmdGait["ForceMode"] = 0
            self.cmdGait["Res"] = 0
            self.symbol_Cmd_Gait.write(self.cmdGait)
            self.cmdPose["X"] = min(400, target_pose.position.x * 1000)
            self.cmdPose["Y"] = min(200, target_pose.position.y * 1000)
            self.cmdPose["Z"] = target_pose.position.z * 1000
            euler = euler_from_quaternion([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ])
            self.cmdPose["Roll"] = euler[0]
            self.cmdPose["Pitch"] = euler[1]
            self.cmdPose["Yaw"] = euler[2]
            self.cmdPose["FG"] = 0
            self.cmdPose["Res"] = 0
            self.symbol_Cmd_Pose.write(self.cmdPose)
            self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)
            rospy.loginfo(f"Started movement to pose: {target_pose.position}")
            return True
        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
            return False

    def move_to_pos(self, cur_pose: Pose, aim_pose: PoseStamped, use_virtual_odom):
        if not self._read_plc_parameters():
            return False
        self.cmdTime["TA"] = 1.5
        self.cmdTime["TM"] = 1.5
        self.cmdTime["TD"] = 0.0
        self.cmdTime["TZ"] = 0.2
        self.symbol_Cmd_Time.write(self.cmdTime)
        self.cmdGait["GaitMode"] = 1
        self.cmdGait["GaitDF"] = 0.5
        self.cmdGait["SwapHigh"] = 100.0
        self.cmdGait["LegNum"] = 0
        self.cmdGait["ForceMode"] = 0
        self.cmdGait["Res"] = 0
        self.symbol_Cmd_Gait.write(self.cmdGait)
        self.cmdPose["X"] = max(-380, min(380, (aim_pose.pose.position.x - cur_pose.position.x) * 1000))
        self.cmdPose["Y"] = max(-100, min(100, (aim_pose.pose.position.y - cur_pose.position.y) * 1000))
        self.cmdPose["Z"] = max(-50, min(50, (aim_pose.pose.position.z - cur_pose.position.z) * 1000))
        print(f"[move to] cmdPose set is: x: {(aim_pose.pose.position.x - cur_pose.position.x) * 1000}, y: {(aim_pose.pose.position.y - cur_pose.position.y) * 1000}")
        print(f"[move to] cmdPose set is clamped to: x: {self.cmdPose['X']}, y: {self.cmdPose['Y']}")
        if use_virtual_odom:
            self.current_pose.position.x += max(-0.38, min(0.38, aim_pose.pose.position.x - cur_pose.position.x))
            self.current_pose.position.y += max(-0.1, min(0.1, aim_pose.pose.position.y - cur_pose.position.y))
            self.current_pose.position.z = aim_pose.pose.position.z
        self.cmdPose["Roll"] = 0.0
        self.cmdPose["Pitch"] = 0.0
        self.cmdPose["Yaw"] = 0.0
        self.cmdPose["FG"] = 0
        self.cmdPose["Res"] = 0
        self.symbol_Cmd_Pose.write(self.cmdPose)
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0:
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)
        return True

    def move_to_yaw(self, cur_pose: Pose, aim_pose: PoseStamped, use_virtual_odom):
        if not self._read_plc_parameters():
            return False
        self.cmdTime["TA"] = 1.5
        self.cmdTime["TM"] = 1.5
        self.cmdTime["TD"] = 0.0
        self.cmdTime["TZ"] = 0.2
        self.symbol_Cmd_Time.write(self.cmdTime)
        self.cmdGait["GaitMode"] = 1
        self.cmdGait["GaitDF"] = 0.5
        self.cmdGait["SwapHigh"] = 100.0
        self.cmdGait["LegNum"] = 0
        self.cmdGait["ForceMode"] = 0
        self.cmdGait["Res"] = 0
        self.symbol_Cmd_Gait.write(self.cmdGait)
        self.cmdPose["X"] = 0.0
        self.cmdPose["Y"] = 0.0
        self.cmdPose["Z"] = 0.0
        self.cmdPose["Roll"] = 0.0
        self.cmdPose["Pitch"] = 0.0
        yaw_diff: float = self.calPosYawDiff2d(aim_pose.pose, self.current_pose)
        yaw_control = -yaw_diff
        self.cmdPose["Yaw"] = max(-5.0 / 180.0 * math.pi, min(5.0 / 180.0 * math.pi, yaw_control))
        print(f"[move to] cmdPose set is: yaw: {yaw_control}")
        print(f"[move to] cmdPose set is clamped to: yaw: {self.cmdPose['Yaw']}")
        if use_virtual_odom:
            last_yaw: float = self.calPosYaw2d(self.current_pose)
            cur_yaw: float = last_yaw + self.cmdPose['Yaw']
            cur_yaw_quant = self._calQuanfromyaw(cur_yaw)
            self.current_pose.orientation = cur_yaw_quant
        self.cmdPose["FG"] = 0
        self.cmdPose["Res"] = 0
        self.symbol_Cmd_Pose.write(self.cmdPose)
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0:
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)
        return True

    def follow_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value == 7:
                print(f"try to enable plc")
                if self._enable_plc():
                    print("enable plc success, continue")
                else:
                    print("enable plc failed")
            else:
                print(f"error symbol_QState: {self.symbol_QState}, can't enbale PLC, return false")
                return False
        cur_step: int = 0
        while cur_step < len(trajectory.poses):
            aim_pose = trajectory.poses[cur_step]
            print(f"[follow_traj]aim_pose x= {aim_pose.pose.position.x}"
                  f"y= {aim_pose.pose.position.y} z= {aim_pose.pose.position.z}"
                  f"yaw= {self.calPosYaw2d(aim_pose)/math.pi*180.0} cur_step= {cur_step} ")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            if cur_beifu_Cmd["FG"] == CtrlCmd.IDLE:
                cur_pose: Pose = self.get_current_pose()
                print(f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose)) < self.admit_yaw_limit:
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=False)
                    else:
                        print(f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=False)
        finished_msg = Bool()
        finished_msg.data = True
        self.task_status_pub.publish(finished_msg)
        return True

    def incremental_cmd_callback(self, msg: Twist):
        """增量步进指令"""
        if self.is_moving:
            rospy.logwarn("Robot is moving, ignore incremental cmd")
            return
        self.is_moving = True
        try:
            current_state = self.symbol_QState.value
            if current_state == 7:
                rospy.logwarn("PLC is disabled (state=7), trying to enable automatically...")
                if self._enable_plc():
                    rospy.loginfo("PLC enabled successfully for incremental cmd")
                else:
                    rospy.logerr("Failed to enable PLC, abort incremental cmd")
                    return
            elif current_state != 2:
                rospy.logerr("PLC not ready, state=%d (expected 2 or 7)", current_state)
                return
            cur_pose = self.get_current_pose()
            cur_yaw = self.calPosYaw2d(cur_pose)
            dx_body = msg.linear.x
            dy_body = msg.linear.y
            dz_body = msg.linear.z
            d_yaw = msg.angular.z
            if abs(dx_body) < 0.001 and abs(dy_body) < 0.001 and abs(dz_body) < 0.001 and abs(d_yaw) < 0.001:
                rospy.loginfo("Incremental cmd near zero, ignored")
                return
            cos_y = math.cos(cur_yaw)
            sin_y = math.sin(cur_yaw)
            dx_world = dx_body * cos_y - dy_body * sin_y
            dy_world = dx_body * sin_y + dy_body * cos_y
            dz_world = dz_body
            aim_pose = PoseStamped()
            aim_pose.header.frame_id = "odom"
            aim_pose.pose.position.x = cur_pose.position.x + dx_world
            aim_pose.pose.position.y = cur_pose.position.y + dy_world
            aim_pose.pose.position.z = cur_pose.position.z + dz_world
            aim_pose.pose.orientation = cur_pose.orientation
            rospy.loginfo("[IncCmd] dX=%.3f, dY=%.3f, dZ=%.3f, dYaw=%.2f°",
                          dx_body, dy_body, dz_body, math.degrees(d_yaw))
            if abs(d_yaw) > 0.001:
                yaw_target_pose = PoseStamped()
                yaw_target_pose.pose = cur_pose
                self.move_to_yaw(cur_pose, yaw_target_pose, use_virtual_odom=False)
            if abs(dx_body) > 0.001 or abs(dy_body) > 0.001 or abs(dz_body) > 0.001:
                self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=False)
            rospy.loginfo("[IncCmd] Step command finished")
        except Exception as e:
            rospy.logerr("Incremental cmd failed: %s", str(e))
        finally:
            self.is_moving = False

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_virtual_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value == 7:
                print(f"try to enable plc")
                if self._enable_plc():
                    print("enable plc success, continue")
                else:
                    print("enable plc failed")
            else:
                print(f"error symbol_QState: {self.symbol_QState}, cann't enbale PCL, return false")
                return False
        cur_step: int = 0
        while cur_step < len(trajectory.poses):
            aim_pose = trajectory.poses[cur_step]
            print(f"[follow_traj]aim_pose x= {aim_pose.pose.position.x}"
                  f"y= {aim_pose.pose.position.y} z= {aim_pose.pose.position.z}"
                  f"yaw= {self.calPosYaw2d(aim_pose)/math.pi*180.0} cur_step= {cur_step} ")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            if cur_beifu_Cmd["FG"] == CtrlCmd.IDLE:
                cur_pose: Pose = self.get_current_pose()
                print(f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose)) < self.admit_yaw_limit:
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=True)
                    else:
                        print(f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=True)
        return True

    # Free gait methods
    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False
        if not self._enable_plc():
            return False
        if not self._read_plc_parameters():
            return False
        self.cmdTime["TA"] = 1.0
        self.cmdTime["TM"] = 1.5
        self.cmdTime["TD"] = 0.0
        self.cmdTime["TZ"] = 0.6
        self.symbol_Cmd_Time.write(self.cmdTime)
        self.cmdGait["GaitMode"] = 5
        self.cmdGait["GaitDF"] = 0.5
        self.cmdGait["SwapHigh"] = 300.0
        self.cmdGait["LegNum"] = 0
        self.cmdGait["ForceMode"] = 0
        self.cmdGait["Res"] = 0
        self.symbol_Cmd_Gait.write(self.cmdGait)
        self.symbol_CtrlCmd.write(CtrlCmd.REMOTE_MOV)
        timeout = 5.0
        start_time = rospy.Time.now()
        self.symbol_ReqFlag.write(0)
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.symbol_ReqFlag.value == 1:
                break
            rospy.sleep(0.1)
        if self.symbol_ReqFlag.value != 1:
            rospy.logwarn("CPP not ready for free gait command")
            return False
        if self.ReqPTCmd is None:
            self.ReqPTCmd = self.symbol_ReqPTCmd.read()
        if self.keepConstBaseFootZ:
            body_motion[2] = 0.0
        self.ReqPTCmd["X"] = body_motion[0]
        self.ReqPTCmd["Y"] = body_motion[1]
        self.ReqPTCmd["Z"] = body_motion[2]
        self.ReqPTCmd["Roll"] = body_motion[3]
        self.ReqPTCmd["Pitch"] = body_motion[4]
        self.ReqPTCmd["Yaw"] = body_motion[5]
        self.ReqPTCmd["FG"] = 0
        self.ReqPTCmd["Res"] = 0
        for j in range(6):
            i = FOOT_REMAP[j]
            x_key = f"X{i+1}"
            y_key = f"Y{i+1}"
            z_key = f"Z{i+1}"
            sf_key = f"SF{i+1}"
            self.ReqPTCmd[x_key] = foot_positions[j, 0]
            self.ReqPTCmd[y_key] = foot_positions[j, 1]
            self.ReqPTCmd[z_key] = foot_positions[j, 2]
            self.ReqPTCmd[sf_key] = foot_flags[j]
        self.symbol_ReqPTCmd.write(self.ReqPTCmd)
        self.symbol_ReqFlag.write(2)
        self._update_footpos_from_plc()
        self.target_foot_positions = foot_positions.copy()
        self.foot_support_flags = foot_flags.copy()
        rospy.loginfo(f"Started free gait movement with body motion: {body_motion}")
        return True

    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False
        if not self._enable_plc():
            return False
        if not self._read_plc_parameters():
            return False
        pos_diff = np.array([
            (target_pose.position.x - self.current_pose.position.x) * 1000.0,
            (target_pose.position.y - self.current_pose.position.y) * 1000.0,
            (target_pose.position.z - self.current_pose.position.z) * 1000.0
        ])
        current_euler = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ])
        target_euler = euler_from_quaternion([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ])
        angle_diff = np.array([
            target_euler[0] - current_euler[0],
            target_euler[1] - current_euler[1],
            target_euler[2] - current_euler[2]
        ])
        if angle_diff[2] > math.pi:
            angle_diff[2] -= 2 * math.pi
        elif angle_diff[2] < -math.pi:
            angle_diff[2] += 2 * math.pi
        body_motion = np.concatenate([pos_diff, angle_diff])
        success = self.move_free_gait(body_motion, foot_positions, foot_flags)
        if success:
            rospy.loginfo("Coordinated pose and foot movement completed")
        return success

    def setCmd(self, **kwargs) -> bool:
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return False
        try:
            if 'TA' in kwargs:
                self.cmdTime["TA"] = kwargs['TA']
            if 'TM' in kwargs:
                self.cmdTime["TM"] = kwargs['TM']
            if 'TD' in kwargs:
                self.cmdTime["TD"] = kwargs['TD']
            if 'TZ' in kwargs:
                self.cmdTime["TZ"] = kwargs['TZ']
            if 'GaitMode' in kwargs:
                self.cmdGait["GaitMode"] = kwargs['GaitMode']
            if 'GaitDF' in kwargs:
                self.cmdGait["GaitDF"] = kwargs['GaitDF']
            if 'SwapHigh' in kwargs:
                self.cmdGait["SwapHigh"] = kwargs['SwapHigh']
            if 'ForceMode' in kwargs:
                self.cmdGait["ForceMode"] = kwargs['ForceMode']
            if self.cmdTime:
                self.symbol_Cmd_Time.write(self.cmdTime)
            if self.cmdGait:
                self.symbol_Cmd_Gait.write(self.cmdGait)
            rospy.loginfo(f"Movement parameters updated: {kwargs}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set parameters: {str(e)}")
            return False

    def stop_movement(self) -> bool:
        if not self.plc_connected:
            return False
        try:
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)
            rospy.loginfo("Movement stopped")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to stop movement: {str(e)}")
            return False

    def joint_encoder_timer_callback(self, event):
        pass

    def robot_is_moving(self):
        if self.symbol_ReqFlag is not None:
            return not self.symbol_ReqFlag.value
        else:
            return 0

    def test_footpos_read(self):
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return
        try:
            act_pos = self.symbol_PTActPos.read()
            if act_pos:
                rospy.loginfo(f"Current Pose from PLC: X={act_pos['X']}, Y={act_pos['Y']}, Z={act_pos['Z']}, "
                              f"Roll={act_pos['Roll']}, Pitch={act_pos['Pitch']}, Yaw={act_pos['Yaw']}")
            else:
                rospy.logwarn("No pose data received from PLC")
        except Exception as e:
            rospy.logerr(f"Failed to read pose from PLC: {str(e)}")