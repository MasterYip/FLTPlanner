#!/usr/bin/env python3

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
import geometry_msgs.msg  # Add this import for Point and Vector3
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path
import copy
# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle  # pyright: ignore[reportAttributeAccessIssue]
# Import FootState message
from legged_traj_plan.msg import FootState


FOOT_REMAP = [3, 4, 5, 0, 1, 2]  # Remap from FootElAir to Foot201


class Hexapod201Interface(Hexapod201BaseInterface):
    """Real hexapod interface using PLC communication"""
    
    def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
        super().__init__(node_name)
        
        self.joint_encoder_timer = rospy.Timer(rospy.Duration(0, int(2e7)), self.joint_encoder_timer_callback)
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
        self.admit_pose_limit = 0.10 # 0.10m
        self.admit_yaw_limit = 4.0 / 180.0 * math.pi # 4° degree
                
        # Connect to PLC and CPP
        self._connect_plc()
        
        rospy.loginfo("Real hexapod interface initialized")
    
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
        self.symbol_QState.auto_update = True    # Q的意思是输出状态, 状态指当前单步, 连续， 停止还是别的状态
        self.symbol_ReqFlag.auto_update = True
        self.symbol_PTActPos.auto_update = True  # PT的意思是平台
        self.symbol_PTCmdPos.auto_update = True
        self.symbol_QJointPos.auto_update = True
        self.symbol_QPTVel_Force.auto_update = True
        self.symbol_QJointVel.auto_update = True
        
          
        self.plc_connected = True
        self.cpp_connected = True
        rospy.loginfo("PLC and CPP connections established")

    
    def _robot_pose_sub_callback(self, msg: Odometry):
        self.current_pose.position = msg.pose.pose.position
        self.current_pose.orientation = msg.pose.pose.orientation
        print(f"robot current pose updated! "
              f"x: {self.current_pose.position.x}"
              f"x: {self.current_pose.position.y}"
              f"x: {self.current_pose.position.z}")
        
        return
    
    def _enable_plc(self):
        """Enable PLC for movement"""
        if not self.plc_connected:
            return False
        
        try:
            print("start self.symbol_State.write(State.ENABLE)")
            self.symbol_State.write(State.ENABLE) # pyright: ignore[reportOptionalMemberAccess]
            print("finish self.symbol_State.write(State.ENABLE)")
            # Wait for enable
            timeout = 10.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                if self.symbol_QState.value == State.FEEDMOV: # type: ignore
                    rospy.loginfo("PLC enabled successfully")
                    return True
                rospy.sleep(0.1)
            
            rospy.logwarn("PLC enable timeout")
            return False
            
        except Exception as e:
            rospy.logerr(f"PLC enable failed: {str(e)}")
            return False
    
    def _read_plc_parameters(self):
        """Read current PLC parameters"""
        if not self.plc_connected:
            return False
        
        try:
            if self.cmdTime is None:
                self.cmdTime = self.symbol_Cmd_Time.read() # type: ignore
            if self.cmdGait is None:
                self.cmdGait = self.symbol_Cmd_Gait.read() # type: ignore
            if self.cmdPose is None:
                self.cmdPose = self.symbol_Cmd_Pose.read() # type: ignore
            
            return True
        except Exception as e:
            rospy.logerr(f"Failed to read PLC parameters: {str(e)}")
            return False
    
    def _update_current_pose_from_plc(self):
        """Update current pose from PLC feedback"""
        if not self.plc_connected:
            return
        
        try:
            act_pos = self.symbol_PTActPos.read() # type: ignore
            if act_pos:
                self.current_pose.position.x = act_pos["X"]
                self.current_pose.position.y = act_pos["Y"]
                self.current_pose.position.z = act_pos["Z"]
                
                # Convert Euler angles to quaternion
                quat = quaternion_from_euler(act_pos["Roll"], act_pos["Pitch"], act_pos["Yaw"])
                self.current_pose.orientation.w = quat[0]
                self.current_pose.orientation.x = quat[1]
                self.current_pose.orientation.y = quat[2]
                self.current_pose.orientation.z = quat[3]
        except Exception as e:
            rospy.logwarn(f"Failed to read current pose from PLC: {str(e)}")
    
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Move hexapod to target pose using PLC"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return False
        
        try:
            # Enable PLC
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Set movement parameters
            self.cmdTime["TA"] = 1.5  # type: ignore # Acceleration time
            self.cmdTime["TM"] = 1.5  # type: ignore # Movement time
            self.cmdTime["TD"] = 0.0  # type: ignore # Deceleration overlap
            self.cmdTime["TZ"] = 0.2  # type: ignore # Z advance time
            self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
            
            # Set gait parameters
            self.cmdGait["GaitMode"] = 1  # type: ignore # Synchronous gait
            self.cmdGait["GaitDF"] = 0.5  # type: ignore # Duty factor
            self.cmdGait["SwapHigh"] = 100.0  # type: ignore # Swing height (mm)
            self.cmdGait["LegNum"] = 0  # type: ignore # Force control mode
            self.cmdGait["ForceMode"] = 0  # type: ignore 
            self.cmdGait["Res"] = 0  # type: ignore 
            self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore 
            
            # Set pose parameters
            self.cmdPose["X"] = min(400, target_pose.position.x * 1000)  # type: ignore  # Convert to mm
            self.cmdPose["Y"] = min(200, target_pose.position.y * 1000)  # type: ignore 
            self.cmdPose["Z"] = target_pose.position.z * 1000  # type: ignore 
            
            # Convert quaternion to Euler angles
            euler = euler_from_quaternion([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ])
            self.cmdPose["Roll"] = euler[0]  # type: ignore 
            self.cmdPose["Pitch"] = euler[1] # type: ignore 
            self.cmdPose["Yaw"] = euler[2]   # type: ignore 
            
            self.cmdPose["FG"] = 0  # type: ignore   # Movement mode
            self.cmdPose["Res"] = 0 # type: ignore 
            self.symbol_Cmd_Pose.write(self.cmdPose)  # type: ignore 
            
            # Start movement
            self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)  # type: ignore 
            
            rospy.loginfo(f"Started movement to pose: {target_pose.position}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
            return False
    
    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod using free gait with custom foothold definitions"""
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False
        
        try:
            # Enable PLC for free gait
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Set free gait parameters
            self.cmdTime["TA"] = 1.0  # type: ignore # Acceleration time
            self.cmdTime["TM"] = 1.5  # type: ignore # Movement time
            self.cmdTime["TD"] = 0.0  # type: ignore # Deceleration overlap
            self.cmdTime["TZ"] = 0.5  # type: ignore # Z advance time
            self.symbol_Cmd_Time.write(self.cmdTime)
            
            # Set gait parameters for free gait
            self.cmdGait["GaitMode"] = 5  # Free gait mode
            self.cmdGait["GaitDF"] = 0.5  # Duty factor
            self.cmdGait["SwapHigh"] = 80.0  # Swing height (mm)
            self.cmdGait["LegNum"] = 0  # Leg number
            self.cmdGait["ForceMode"] = 0  # Force control mode
            self.cmdGait["Res"] = 0
            self.symbol_Cmd_Gait.write(self.cmdGait)
            
            # Start remote free gait movement
            self.symbol_CtrlCmd.write(CtrlCmd.REMOTE_MOV)
            
            # Wait for CPP to be ready for command
            timeout = 5.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                if self.symbol_ReqFlag.value == 1:
                    break
                rospy.sleep(0.1)
            if self.symbol_ReqFlag.value != 1:
                rospy.logwarn("CPP not ready for free gait command")
                return False
            
            # Prepare free gait command
            if self.ReqPTCmd is None:
                self.ReqPTCmd = self.symbol_ReqPTCmd.read()
            
            # Set body motion
            self.ReqPTCmd["X"] = body_motion[0]  # mm
            self.ReqPTCmd["Y"] = body_motion[1]  # mm
            self.ReqPTCmd["Z"] = body_motion[2]  # mm
            self.ReqPTCmd["Roll"] = body_motion[3]  # rad
            self.ReqPTCmd["Pitch"] = body_motion[4]  # rad
            self.ReqPTCmd["Yaw"] = body_motion[5]  # rad
            self.ReqPTCmd["FG"] = 0  # Movement flag
            self.ReqPTCmd["Res"] = 0  # Reserved
            
            # Set foot positions and flags
            for j in range(6):
                i = FOOT_REMAP[j]
                x_key = f"X{i+1}"
                y_key = f"Y{i+1}"
                z_key = f"Z{i+1}"
                sf_key = f"SF{i+1}"
                
                self.ReqPTCmd[x_key] = foot_positions[j, 0]  # mm
                self.ReqPTCmd[y_key] = foot_positions[j, 1]  # mm
                self.ReqPTCmd[z_key] = foot_positions[j, 2]  # mm
                self.ReqPTCmd[sf_key] = foot_flags[j]  # 0=support, 1=swing
            
            # Send command to CPP
            self.symbol_ReqPTCmd.write(self.ReqPTCmd)
            self.symbol_ReqFlag.write(2)  # Start movement
            
            # Update internal foot positions
            # self.foot_positions = foot_positions.copy()
            self.update_footpos()
            self.target_foot_positions = foot_positions.copy()
            self.foot_support_flags = foot_flags.copy()
            
            rospy.loginfo(f"Started free gait movement with body motion: {body_motion}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Free gait movement failed: {str(e)}")
            return False
            
    def move_to_pos(self, cur_pose: Pose, aim_pose: PoseStamped, use_virtual_odom):
        if not self._read_plc_parameters():
            return False
        # 加载运动参数
        # Set movement parameters
        self.cmdTime["TA"] = 1.5  # type: ignore   # Acceleration time  一步迈过去的加速时间
        self.cmdTime["TM"] = 1.5  # type: ignore   # Movement time
        self.cmdTime["TD"] = 0.0  # type: ignore   # Deceleration overlap
        self.cmdTime["TZ"] = 0.2  # type: ignore   # Z advance time(s)z项提前抬起来
        self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
        
        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # type: ignore    # Synchronous gait 1 是正常的2 3 6 步态
        self.cmdGait["GaitDF"] = 0.5  # type: ignore     # Duty factor 0.5就是2步态 0.667 就是三步态 0.833就是六步态
        self.cmdGait["SwapHigh"] = 100.0  # type: ignore     # Swing height (mm) 摆动高度
        self.cmdGait["LegNum"] = 0  # type: ignore    # Force control mode # 单腿运动的时候控制这个，这个先给0
        self.cmdGait["ForceMode"] = 0 # type: ignore    # 无力控是0
        self.cmdGait["Res"] = 0  # type: ignore        # 给0的话倍福的程序会再算一次足端的步长啥的，所以最好给1
        self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore 
        
        # Set pose parameters
        self.cmdPose["X"] = max(-380, min(380, (aim_pose.pose.position.x - cur_pose.position.x) * 1000)) # type: ignore    # Convert to mm
        self.cmdPose["Y"] = max(-100, min(100, (aim_pose.pose.position.y - cur_pose.position.y) * 1000))  # type: ignore   

        self.cmdPose["Z"] = 0.0  # type: ignore
        print(f"[move to] cmdPose set is: x: {(aim_pose.pose.position.x - cur_pose.position.x) * 1000}, y: {(aim_pose.pose.position.y - cur_pose.position.y) * 1000}")
        print(f"[move to] cmdPose set is clamped to: x: {self.cmdPose['X']}, y: {self.cmdPose['Y']}")
        
        if(use_virtual_odom):
            # 虚拟地将移动后的期望位置更新为机器人的当前位置
            self.current_pose.position.x += max(-0.38, min(0.38, aim_pose.pose.position.x - cur_pose.position.x))
            self.current_pose.position.y += max(-0.1, min(0.1, aim_pose.pose.position.y - cur_pose.position.y))
            self.current_pose.position.z = aim_pose.pose.position.z
        # Convert quaternion to Euler angles
        # euler = euler_from_quaternion([
        #     target_pose.orientation.x,
        #     target_pose.orientation.y,
        #     target_pose.orientation.z,
        #     target_pose.orientation.w
        # ])
        self.cmdPose["Roll"] = 0.0  # type: ignore   
        self.cmdPose["Pitch"] = 0.0 # type: ignore   
        self.cmdPose["Yaw"] = 0.0   # type: ignore   
        
        self.cmdPose["FG"] = 0  # type: ignore   # Movement mode 在发送的时候代表走一步停一下，1代表连续走
        self.cmdPose["Res"] = 0 # type: ignore    # Res = 0, 代表自动计算步长等参数
        self.symbol_Cmd_Pose.write(self.cmdPose) # type: ignore    # write相当于发送
        
        # Start movement
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV) # type: ignore
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0: # type: ignore
            # print(f"cur_beifu_Cmd['FG'] != 0, sendCtrlCmd.STOP_MOV, cur_beifu_Cmd['FG'] is {cur_beifu_Cmd['FG']}")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV) # type: ignore
        
        # rospy.loginfo(f"Started movement to pose: {target_pose.position}")
        return True
    
    def move_to_yaw(self, cur_pose: Pose, aim_pose: PoseStamped, use_virtual_odom):
        if not self._read_plc_parameters():
            return False
        # 加载运动参数
        # Set movement parameters
        self.cmdTime["TA"] = 1.5  # type: ignore   # Acceleration time  一步迈过去的加速时间
        self.cmdTime["TM"] = 1.5  # type: ignore   # Movement time
        self.cmdTime["TD"] = 0.0  # type: ignore   # Deceleration overlap
        self.cmdTime["TZ"] = 0.2  # type: ignore   # Z advance time(s)z项提前抬起来
        self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
        
        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # type: ignore    # Synchronous gait 1 是正常的2 3 6 步态
        self.cmdGait["GaitDF"] = 0.5  # type: ignore     # Duty factor 0.5就是2步态 0.667 就是三步态 0.833就是六步态
        self.cmdGait["SwapHigh"] = 100.0  # type: ignore     # Swing height (mm) 摆动高度
        self.cmdGait["LegNum"] = 0  # type: ignore    # Force control mode # 单腿运动的时候控制这个，这个先给0
        self.cmdGait["ForceMode"] = 0 # type: ignore    # 无力控是0
        self.cmdGait["Res"] = 0  # type: ignore        # 给0的话倍福的程序会再算一次足端的步长啥的，所以最好给1
        self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore 
        
        # Set pose parameters
        self.cmdPose["X"] = 0.0 # type: ignore    # Convert to mm
        self.cmdPose["Y"] = 0.0  # type: ignore   
        self.cmdPose["Z"] = 0.0  # type: ignore
        # Convert quaternion to Euler angles
        # euler = euler_from_quaternion([
        #     target_pose.orientation.x,
        #     target_pose.orientation.y,
        #     target_pose.orientation.z,
        #     target_pose.orientation.w
        # ])
        self.cmdPose["Roll"] = 0.0  # type: ignore   
        self.cmdPose["Pitch"] = 0.0 # type: ignore   
        yaw_diff:float = self.calPosYawDiff2d(aim_pose.pose, self.current_pose)
        yaw_control = -yaw_diff
        self.cmdPose["Yaw"] = max(-5.0/180.0*math.pi, min(5.0/180.0*math.pi, yaw_control))   # type: ignore   
    
        print(f"[move to] cmdPose set is: yaw: {yaw_control}")
        print(f"[move to] cmdPose set is clamped to: yaw: {self.cmdPose['Yaw']}")         # type: ignore   
        
        if(use_virtual_odom):
            # 虚拟地将移动后的期望位置更新为机器人的当前位置
            last_yaw:float =  self.calPosYaw2d(self.current_pose)
            cur_yaw:float = last_yaw + self.cmdPose['Yaw']
            cur_yaw_quant = self._calQuanfromyaw(cur_yaw)
            self.current_pose.orientation = cur_yaw_quant


        self.cmdPose["FG"] = 0  # type: ignore   # Movement mode 在发送的时候代表走一步停一下，1代表连续走
        self.cmdPose["Res"] = 0 # type: ignore    # Res = 0, 代表自动计算步长等参数
        self.symbol_Cmd_Pose.write(self.cmdPose) # type: ignore    # write相当于发送
        
        # Start movement
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV) # type: ignore
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0: # type: ignore
            # print(f"cur_beifu_Cmd['FG'] != 0, sendCtrlCmd.STOP_MOV, cur_beifu_Cmd['FG'] is {cur_beifu_Cmd['FG']}")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV) # type: ignore
        
        # rospy.loginfo(f"Started movement to pose: {target_pose.position}")
        return True
    
    def follow_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:  # type: ignore
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value  == 7:  # type: ignore   
                print(f"try to enable plc")
                if self._enable_plc():
                    print("enable plc success, continue")
                else:
                    print("enable plc failed")
            else:
                print(f"error symbol_QState: {self.symbol_QState}, can't enbale PLC, return false")
                return False
        cur_step:int = 0
        while cur_step < len(trajectory.poses):
            aim_pose = trajectory.poses[cur_step]
            print(f"[follow_traj]aim_pose x= {aim_pose.pose.position.x}"
                  f"y= {aim_pose.pose.position.y} z= {aim_pose.pose.position.z}"
                  f"yaw= {self.calPosYaw2d(aim_pose)/math.pi*180.0} cur_step= {cur_step} ")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            if cur_beifu_Cmd["FG"]==CtrlCmd.IDLE:
                # 拿一下机器人当前的位置
                cur_pose:Pose = self.get_current_pose()
                print(f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose))<self.admit_yaw_limit:
                    # 说明已经到达了当前点
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    # 如果有yaw角度diff 优先动yaw
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=False)
                    else:
                        print(f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=False)
            
        return True
    
    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_virtual_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:  # type: ignore
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value  == 7:  # type: ignore
                print(f"try to enable plc")
                if self._enable_plc():
                    print("enable plc success, continue")
                else:
                    print("enable plc failed")
            else:
                print(f"error symbol_QState: {self.symbol_QState}, cann't enbale PCL, return false")
                return False
        cur_step:int = 0
        while cur_step < len(trajectory.poses):
            aim_pose = trajectory.poses[cur_step]
            print(f"[follow_traj]aim_pose x= {aim_pose.pose.position.x}"
                  f"y= {aim_pose.pose.position.y} z= {aim_pose.pose.position.z}"
                  f"yaw= {self.calPosYaw2d(aim_pose)/math.pi*180.0} cur_step= {cur_step} ")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            if cur_beifu_Cmd["FG"]==CtrlCmd.IDLE:
                # 拿一下机器人当前的位置
                cur_pose:Pose = self.get_current_pose()
                print(f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                # 模拟实际的SLAM反馈中到达目标点的判断函数, 认为走出一步之后就到达了目标点
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose))<self.admit_yaw_limit:
                    # 说明已经到达了当前点
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    # 如果有yaw角度diff 优先动yaw
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=True)
                    else:
                        print(f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=True)

        return True
    
    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod to target pose with specific foot positions using free gait"""
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False
        
        try:
            # Enable PLC for free gait
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Calculate body motion from current to target pose
            # FIXME: current pose should be read from odom
            pos_diff = np.array([
                (target_pose.position.x - self.current_pose.position.x) * 1000.0,  # Convert to mm
                (target_pose.position.y - self.current_pose.position.y) * 1000.0,
                (target_pose.position.z - self.current_pose.position.z) * 1000.0
            ])
            
            # Get current and target Euler angles
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
            
            # Normalize yaw angle difference
            if angle_diff[2] > math.pi:
                angle_diff[2] -= 2 * math.pi
            elif angle_diff[2] < -math.pi:
                angle_diff[2] += 2 * math.pi
            
            # Create body motion array
            body_motion = np.concatenate([pos_diff, angle_diff])
            
            # Use the existing free gait method with calculated body motion
            success = self.move_free_gait(body_motion, foot_positions, foot_flags)
            
            if success:
                # Update current pose to target pose
                # FIXME: this should be updated from odom
                self.current_pose = copy.deepcopy(target_pose)
                rospy.loginfo("Coordinated pose and foot movement completed")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"Coordinated movement failed: {str(e)}")
            return False

    def setCmd(self, **kwargs) -> bool:
        """Set detailed movement parameters"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return False
        
        try:
            # Time parameters
            if 'TA' in kwargs:
                self.cmdTime["TA"] = kwargs['TA'] # type: ignore
            if 'TM' in kwargs:
                self.cmdTime["TM"] = kwargs['TM'] # type: ignore
            if 'TD' in kwargs:
                self.cmdTime["TD"] = kwargs['TD'] # type: ignore
            if 'TZ' in kwargs:
                self.cmdTime["TZ"] = kwargs['TZ'] # type: ignore
            
            # Gait parameters
            if 'GaitMode' in kwargs:
                self.cmdGait["GaitMode"] = kwargs['GaitMode'] # type: ignore
            if 'GaitDF' in kwargs:
                self.cmdGait["GaitDF"] = kwargs['GaitDF'] # type: ignore
            if 'SwapHigh' in kwargs:
                self.cmdGait["SwapHigh"] = kwargs['SwapHigh'] # type: ignore
            if 'ForceMode' in kwargs:
                self.cmdGait["ForceMode"] = kwargs['ForceMode'] # type: ignore
            
            # Write parameters to PLC
            if self.cmdTime:
                self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
            if self.cmdGait:
                self.symbol_Cmd_Gait.write(self.cmdGait) # type: ignore
            
            rospy.loginfo(f"Movement parameters updated: {kwargs}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to set parameters: {str(e)}")
            return False
    
    def stop_movement(self) -> bool:
        """Stop current movement"""
        if not self.plc_connected:
            return False
        
        try:
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV) # type: ignore
            rospy.loginfo("Movement stopped")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to stop movement: {str(e)}")
            return False
    def joint_encoder_timer_callback(self, event):
        """Default callback for joint encoder timer."""
        # joint_values: List[float]
        # joint_values = self.symbol_QJointPos.read()
        # self.cmdPose = self.symbol_Cmd_Pose.read()
        # zhicheng1:int = self.cmdPose["SF1"]
        # zuduanli1:int = self.symbol_QPTVel_Force.read()["Z1"] # ["Z1"]要不就read 
        # jishensudux =self.symbol_QPTVel_Force.value["X"]
        # jishensuduy =self.symbol_QPTVel_Force.value["Y"]
        # guanjiesudu = self.symbol_QJointVel.value[""]
        
        
        rospy.loginfo("Joint encoder timer callback triggered")

    def cleanup(self):
        """Cleanup PLC and CPP connections"""
        if self.plc_connected and self.plc:
            try:
                self.symbol_State.write(State.DISENABLE) # type: ignore
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

    def test_footpos_read(self):
        """Test reading foot positions from PLC"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return
        
        try:
            act_pos = self.symbol_PTActPos.read()
            if act_pos:
                rospy.loginfo(f"Current Pose from PLC: X={act_pos['X']}, Y={act_pos['Y']}, Z={act_pos['Z']}, Roll={act_pos['Roll']}, Pitch={act_pos['Pitch']}, Yaw={act_pos['Yaw']}")
            else:
                rospy.logwarn("No pose data received from PLC")
        except Exception as e:
            rospy.logerr(f"Failed to read pose from PLC: {str(e)}")

    def update_footpos(self):
        """Update foot positions from PLC feedback"""
        if not self.plc_connected:
            return
        
        try:
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
                print("Act Pos", act_pos)
        except Exception as e:
            rospy.logwarn(f"Failed to update foot positions from PLC: {str(e)}")
