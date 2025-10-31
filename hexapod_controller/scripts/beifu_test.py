#!/usr/bin/env python3

import rospy
import numpy as np
import pyads
import time
import threading
import math
from typing import Dict, Any, Optional, Tuple, List
from enum import IntEnum
from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist, PoseStamped, Pose, _PoseStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path
import copy
# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle # pyright: ignore[reportAttributeAccessIssue]


# 运动模式枚举类
class CtrlCmd(IntEnum):
    IDLE = 0
    SET_PTPOSE = 1
    MODAL_MOV = 2
    FORCE_MOV = 3
    BACK_MOV = 4
    FOLLOW_MOV = 6
    WHEEL2LEG = 7
    LEG2WHEEL = 8
    WAIT_TRIG = 9
    EXAMPLE_MOV = 10
    POSE_MOV = 11
    STEP_MOV = 12
    TRACK_MOV = 13
    ONLINE_MOV = 14
    LEGS_MOV = 15
    FORCE_STEP_MOV = 16
    FORCE_ONLINE_MOV = 17
    STOP_MOV = 18
    PARK_MOV = 19
    DITCH_MOV = 20
    OBSTC_MOV = 21
    SLOPE1_MOV = 22
    SLOPE2_MOV = 23
    REMOTE_MOV = 30


# 状态切换枚举类
class State(IntEnum):
    INIT = 0
    ENABLE = 1
    FEEDMOV = 2
    DISENABLE = 3
    SETPOSITION = 4
    RESET = 5
    ERROR = 6
    IDLE = 7


# 运动参数-时间结构定义
stTime_def = (
    ("TA", pyads.PLCTYPE_REAL, 1),
    ("TM", pyads.PLCTYPE_REAL, 1),
    ("TD", pyads.PLCTYPE_REAL, 1),
    ("TZ", pyads.PLCTYPE_REAL, 1),
)

# 运动参数-步态结构定义
stGait_def = (
    ("GaitMode", pyads.PLCTYPE_UDINT, 1),
    ("GaitDF", pyads.PLCTYPE_REAL, 1),
    ("SwapHigh", pyads.PLCTYPE_REAL, 1),
    ("LegNum", pyads.PLCTYPE_UDINT, 1),
    ("ForceMode", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),
)

# 运动参数-步长/姿态/足端结构定义
stPose_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("Roll", pyads.PLCTYPE_REAL, 1),
    ("Pitch", pyads.PLCTYPE_REAL, 1),
    ("Yaw", pyads.PLCTYPE_REAL, 1),
    ("FG", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),

    ("X1", pyads.PLCTYPE_REAL, 1),
    ("Y1", pyads.PLCTYPE_REAL, 1),
    ("Z1", pyads.PLCTYPE_REAL, 1),
    ("SF1", pyads.PLCTYPE_DINT, 1),

    ("X2", pyads.PLCTYPE_REAL, 1),
    ("Y2", pyads.PLCTYPE_REAL, 1),
    ("Z2", pyads.PLCTYPE_REAL, 1),
    ("SF2", pyads.PLCTYPE_DINT, 1),

    ("X3", pyads.PLCTYPE_REAL, 1),
    ("Y3", pyads.PLCTYPE_REAL, 1),
    ("Z3", pyads.PLCTYPE_REAL, 1),
    ("SF3", pyads.PLCTYPE_DINT, 1),

    ("X4", pyads.PLCTYPE_REAL, 1),
    ("Y4", pyads.PLCTYPE_REAL, 1),
    ("Z4", pyads.PLCTYPE_REAL, 1),
    ("SF4", pyads.PLCTYPE_DINT, 1),

    ("X5", pyads.PLCTYPE_REAL, 1),
    ("Y5", pyads.PLCTYPE_REAL, 1),
    ("Z5", pyads.PLCTYPE_REAL, 1),
    ("SF5", pyads.PLCTYPE_DINT, 1),

    ("X6", pyads.PLCTYPE_REAL, 1),
    ("Y6", pyads.PLCTYPE_REAL, 1),
    ("Z6", pyads.PLCTYPE_REAL, 1),
    ("SF6", pyads.PLCTYPE_DINT, 1),
)


class Hexapod201BaseInterface(ABC):
    """Base interface for hexapod control"""
    
    def __init__(self, node_name: str = "hexapod201_interface"):
        self.node_name = node_name
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.w = 1.0
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        
        self.target_pose = Pose()
        self.target_pose.position.x = 0.0
        self.target_pose.position.y = 0.0
        self.target_pose.position.z = 0.0
        self.target_pose.orientation.w = 1.0
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = 0.0
        
        self.is_moving = False
        self.last_cmd_time = rospy.Time.now()
        self.cmd_vel_integration = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        
        # ROS node should be initialized before creating this class
        
        # Publishers and subscribers
        self.pose_pub = rospy.Publisher('/hexapod/current_pose', PoseStamped, queue_size=10)
        # self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/pose_cmd', PoseStamped, self.pose_cmd_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/path_cmd', Path, self.follow_virtual_trajectory)
        
        # Timer for publishing current pose
        self.pose_timer = rospy.Timer(rospy.Duration(0, int(1e8)), self.publish_current_pose)
        self.dt = 1.0
        # Visualization
        self.visualizer = ROSVisualizer("world", "hexapod_visualization")
        
        rospy.loginfo(f"{node_name} initialized")
    
    # 接收线速度和角速度指令, 根据时间步长dt积分为期望的位置增量，加上当前位置求出期望位置, 之后
    # 调用self.move_to_pose()函数执行运动
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands by integrating to get target pose"""
        # current_time = rospy.Time.now()
        # dt = (current_time - self.last_cmd_time).to_sec()
        # self.last_cmd_time = current_time
        dt = self.dt  # Use fixed dt for simplicity
        # Integrate velocity to get position change
        linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        # 将线速度和角速度按照时间步长dt进行积分
        self.cmd_vel_integration[:3] = linear_vel * dt
        self.cmd_vel_integration[3:] = angular_vel * dt
        
        # 计算dt时间之后的期望目标位置
        self.target_pose.position.x = self.current_pose.position.x + self.cmd_vel_integration[0]
        self.target_pose.position.y = self.current_pose.position.y + self.cmd_vel_integration[1]
        self.target_pose.position.z = self.current_pose.position.z + self.cmd_vel_integration[2]
        
        # Convert Euler angles to quaternion
        # FIXME: 这里可能不正确
        # BUG: this is not correct
        roll, pitch, yaw = self.cmd_vel_integration[3], self.cmd_vel_integration[4], self.cmd_vel_integration[5]
        quat = quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.orientation.w = quat[3]
        self.target_pose.orientation.x = quat[0]
        self.target_pose.orientation.y = quat[1]
        self.target_pose.orientation.z = quat[2]

        # Execute movement
        self.move_to_pose(self.target_pose)
    
    # 直接给出目标位置, 然后调用self.move_to_pose()函数执行运动
    def pose_cmd_callback(self, msg: PoseStamped):
        """Handle direct pose commands"""
        quat = np.array([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ])
        rospy.loginfo(f"Dummy received pose_cmd x: {self.current_pose.position.x:.2f}, "
                      f" y: {self.current_pose.position.y:.2f}, "
                      f" z: {self.current_pose.position.z:.2f}, "
                      f" r: {quat[0]*180/3.1415926535:.2f},"
                      f" p: {quat[1]*180/3.1415926535:.2f},"
                      f" y: {quat[2]*180/3.1415926535:.2f}")
        self.target_pose = msg.pose
        self.move_to_pose(self.target_pose)
    
    # 发布当前位置, 将当前位置在odom坐标系下发布, 并可视化一个长方体形状的六足机体
    def publish_current_pose(self, event):
        """Publish current pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)
        
        # Visualize hexapod body as a box
        self.visualize_hexapod_body()
    
    def visualize_hexapod_body(self):
        """Visualize hexapod body as a box in RViz"""
        # Clear previous visualization
        # self.visualizer.del_cube()
        self.visualizer.del_all()
        
        # Create box at current pose
        position = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        # Convert quaternion to rotation matrix (simplified)
        quat = np.array([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ])
        rpy =  euler_from_quaternion(quat)
        heading_vec = np.array([math.cos(rpy[2]), math.sin(rpy[2]), 0.0])  # Heading direction in XY plane
        # Box size (hexapod body dimensions)
        box_size = [0.6, 0.3, 0.2]  # 30cm cube
        self.visualizer.vis_cube(position, quat, VisStyle(1.0, 0.45, 0.0, 1.0, box_size[0], box_size[1], box_size[2]))
        self.visualizer.vis_arrow(position, position + heading_vec * 0.4)
    
    @abstractmethod
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Move hexapod to target pose - to be implemented by subclasses"""
        pass
    
    @abstractmethod
    def follow_trajectory(self, trajectory: Path) -> bool:
        """Move hexapod to target pose - to be implemented by subclasses"""
        pass
    
    @abstractmethod
    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        """Move hexapod to target pose - to be implemented by subclasses"""
        pass
    
    @abstractmethod
    def setCmd(self, **kwargs) -> bool:
        """Set detailed command parameters - to be implemented by subclasses"""
        pass
    
    @abstractmethod
    def stop_movement(self) -> bool:
        """Stop current movement - to be implemented by subclasses"""
        pass
    
    def reset_integration(self):
        """Reset velocity integration"""
        self.cmd_vel_integration = np.zeros(6)
    
    def get_current_pose(self) -> Pose:
        """Get current pose"""
        return self.current_pose
    
    def get_target_pose(self) -> Pose:
        """Get target pose"""
        return self.target_pose


class DummyHexapod201Interface(Hexapod201BaseInterface):
    """Dummy interface for simulation/testing"""

    def cleanup(self):
        """Dummy cleanup method"""
        rospy.loginfo("Dummy interface cleanup called")
    
    def __init__(self, node_name: str = "dummy_hexapod201_interface"):
        super().__init__(node_name)
        self.movement_lock = threading.Lock()  # Add this line to define movement_lock

    def follow_trajectory(self, trajectory: Path) -> bool:
        """Simulate following a trajectory"""
        rospy.loginfo("Dummy interface following trajectory")
        return True

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        """Simulate virtual following of a trajectory"""
        rospy.loginfo("Dummy interface virtually following trajectory")
        return True
        self.movement_speed = 1.0  # m/s
        self.rotation_speed = 0.5  # rad/s # FIXME:
        self.movement_thread = None
        self.movement_lock = threading.Lock()
        
        rospy.loginfo("Dummy hexapod interface initialized")
    
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Simulate movement to target pose"""
        with self.movement_lock:
            if self.is_moving:
                rospy.logwarn("Movement already in progress")
                return False
            
            self.is_moving = True
            self.target_pose = target_pose
            
            # Start movement in separate thread
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join()
            
            self.movement_thread = threading.Thread(target=self._execute_movement)
            self.movement_thread.start()
            
            return True
    
    def _execute_movement(self):
        """Execute movement in blocking manner"""
        try:
            start_pose = copy.deepcopy(self.current_pose)
            target_pose = self.target_pose
            
            # Calculate distance and angle differences
            pos_diff = np.array([
                target_pose.position.x - start_pose.position.x,
                target_pose.position.y - start_pose.position.y,
                target_pose.position.z - start_pose.position.z
            ])
            
            # Get current and target Euler angles
            # FIXME: 这里需要确认这个得到的euler是什么顺序, r, p, y吗
            # 需要确认一下是不是弧度的
            start_euler = euler_from_quaternion([
                start_pose.orientation.x,
                start_pose.orientation.y,
                start_pose.orientation.z,
                start_pose.orientation.w,
            ])
            
            target_euler = euler_from_quaternion([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ])
            angle_diff = np.array([
                target_euler[0] - start_euler[0],
                target_euler[1] - start_euler[1],
                target_euler[2] - start_euler[2]
            ])
            # FIXME: 确实，这里只有yaw应该生效, 但是这种方式也比较粗暴
            while(angle_diff[2] > math.pi):
                angle_diff[2] -= 2 * math.pi
            while(angle_diff[2] < -math.pi):
                angle_diff[2] += 2 * math.pi

            
            # Calculate movement time
            # FIXME: 这里求的是
            pos_distance = np.linalg.norm(pos_diff)
            angle_distance = np.linalg.norm(angle_diff)
            
            pos_time = pos_distance / self.movement_speed if pos_distance > 0 else 0
            print("pos time = ", pos_time)
            angle_time = angle_distance / self.rotation_speed if angle_distance > 0 else 0
            print("angle time = ", angle_time)
            total_time = max(pos_time, angle_time)
            print("total time = ", total_time)
            
            if total_time > 0:
                start_time = rospy.Time.now()
                rate = rospy.Rate(50)  # 50 Hz update rate
                
                while not rospy.is_shutdown():
                    current_time = rospy.Time.now()
                    elapsed = (current_time - start_time).to_sec()
                    progress = min(elapsed / total_time, 1.0)

                    # Interpolate position
                    self.current_pose.position.x = start_pose.position.x + pos_diff[0] * progress
                    self.current_pose.position.y = start_pose.position.y + pos_diff[1] * progress
                    self.current_pose.position.z = start_pose.position.z + pos_diff[2] * progress

                    # Interpolate orientation
                    current_euler = start_euler + angle_diff * progress
                    quat = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
                    self.current_pose.orientation.w = quat[3]
                    self.current_pose.orientation.x = quat[0]
                    self.current_pose.orientation.y = quat[1]
                    self.current_pose.orientation.z = quat[2]
                    # rospy.loginfo(
                    #     f"Dummy has moved to x: {self.current_pose.position.x:.2f}, "
                    #     f"y: {self.current_pose.position.y:.2f}, "
                    #     f"z: {self.current_pose.position.z:.2f}, "
                    #     f"r: {current_euler[0]:.2f}, p: {current_euler[1]:.2f}, y: {current_euler[2]:.2f}"
                    # )
                    
                    if progress >= 1.0:
                        break
                    
                    rate.sleep()
            
            rospy.loginfo("Dummy movement completed")
            
        except Exception as e:
            rospy.logerr(f"Error in dummy movement: {str(e)}")
        finally:
            with self.movement_lock:
                self.is_moving = False
    
    def setCmd(self, **kwargs) -> bool:
        """Set dummy interface parameters"""
        if 'movement_speed' in kwargs:
            self.movement_speed = kwargs['movement_speed']
        if 'rotation_speed' in kwargs:
            self.rotation_speed = kwargs['rotation_speed']
        
        rospy.loginfo(f"Dummy interface parameters updated: {kwargs}")
        return True
    
    def stop_movement(self) -> bool:
        """Stop dummy movement"""
        with self.movement_lock:
            self.is_moving = False
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join(timeout=1.0)
        
        rospy.loginfo("Dummy movement stopped")
        return True


class Hexapod201Interface(Hexapod201BaseInterface):
    """Real hexapod interface using PLC communication"""
    
    def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
        super().__init__(node_name)
        
        self.robot_pose_sub = rospy.Subscriber('/Odometry', Odometry, self._robot_pose_sub_callback)
        self.plc_ip = plc_ip
        self.plc = None
        self.plc_connected = False
        
        # PLC symbols
        self.symbol_Cmd_Time = None
        self.symbol_Cmd_Gait = None
        self.symbol_Cmd_Pose = None
        self.symbol_CtrlCmd = None
        self.symbol_State = None
        self.symbol_QState = None
        self.symbol_PTActPos = None
        
        # Movement parameters
        self.cmdTime = None
        self.cmdGait = None
        self.cmdPose = None
        
        # 判断一个位置到了没有的容许误差
        self.admit_pose_limit = 0.10 # 0.10m
        self.admit_angle_limit = 4.0 / 180.0 * math.pi # 4° degree
        
        # Connect to PLC
        self._connect_plc()
        
        rospy.loginfo("Real hexapod interface initialized")
    
    def _connect_plc(self):
        """Establish PLC connection"""
        try:
            self.plc = pyads.Connection(self.plc_ip, pyads.PORT_TC3PLC1, '192.168.3.101')
            self.plc.open()
            
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
            # Initialize CPP symbols for free gait
            # self.symbol_ReqPTCmd = self.cpp.get_symbol('CPP.Inputs.ReqPTCmd', structure_def=stPose_def)
            # self.symbol_ReqFlag = self.cpp.get_symbol('CPP.Inputs.ReqFlag')
            
            
            # Enable auto-update for feedback
            self.symbol_QState.auto_update = True    # Q的意思是输出状态, 状态指当前单步, 连续， 停止还是别的状态
            # self.symbol_ReqFlag.auto_update = True
            self.symbol_PTActPos.auto_update = True  # PT的意思是平台
            self.symbol_PTCmdPos.auto_update = True
            
            self.plc_connected = True
            rospy.logdebug("PLC connection established......................................................")
            rospy.logdebug("PLC connection established")
            rospy.logdebug("PLC connection established")
            rospy.logdebug("PLC connection established")
            rospy.loginfo("PLC and CPP connections established")
            
        except Exception as e:
            rospy.logerr(f"PLC connection failed: {str(e)}")
            self.plc_connected = False
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
            self.cmdTime["TA"] = 0.5  # type: ignore # Acceleration time
            self.cmdTime["TM"] = 1.5  # type: ignore # Movement time
            self.cmdTime["TD"] = 0.0  # type: ignore # Deceleration overlap
            self.cmdTime["TZ"] = 0.0  # type: ignore # Z advance time
            self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
            
            # Set gait parameters
            self.cmdGait["GaitMode"] = 1  # type: ignore # Synchronous gait
            self.cmdGait["GaitDF"] = 0.5  # type: ignore # Duty factor
            self.cmdGait["SwapHigh"] = 80.0  # type: ignore # Swing height (mm)
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
    def calPosDisDiff3d(self, a:Pose, b:Pose):
        return pow(pow(a.position.x - b.position.x, 2)+
                   pow(a.position.y - b.position.y, 2)+
                   pow(a.position.z - b.position.z, 2), 0.5)
    def calPosDisDiff2d(self, a:Pose, b:Pose):
        return pow(pow(a.position.x - b.position.x, 2)+
                    pow(a.position.y - b.position.y, 2), 0.5)
    def calPosYawDiff2d(self, a: Pose, b: Pose) -> float:
        """Calculate the angular difference (yaw) between two poses in 2D."""
        quat_a = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
        quat_b = [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w]
    
        # Convert quaternions to Euler angles
        _, _, yaw_a = euler_from_quaternion(quat_a)
        _, _, yaw_b = euler_from_quaternion(quat_b)
    
        # Calculate angular difference
        angle_diff = yaw_b - yaw_a
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
    
        return angle_diff
    
    def calPosYaw2d(self, a: Pose) -> float:
        """Calculate the angular difference (yaw) between two poses in 2D."""
        quat_a = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
    
        # Convert quaternions to Euler angles
        _, _, yaw_a = euler_from_quaternion(quat_a)
        
        while yaw_a > math.pi:
            yaw_a -= 2 * math.pi
        while yaw_a < -math.pi:
            yaw_a += 2 * math.pi
        return yaw_a
        
    def move_to_pos(self, cur_pose: Pose, aim_pose: PoseStamped):
        if not self._read_plc_parameters():
            return False
        # 加载运动参数
        # Set movement parameters
        self.cmdTime["TA"] = 1.5  # type: ignore   # Acceleration time  一步迈过去的加速时间
        self.cmdTime["TM"] = 2.5  # type: ignore   # Movement time
        self.cmdTime["TD"] = 0.0  # type: ignore   # Deceleration overlap
        self.cmdTime["TZ"] = 0.2  # type: ignore   # Z advance time(s)z项提前抬起来
        self.symbol_Cmd_Time.write(self.cmdTime) # type: ignore
        
        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # type: ignore    # Synchronous gait 1 是正常的2 3 6 步态
        self.cmdGait["GaitDF"] = 0.5  # type: ignore     # Duty factor 0.5就是2步态 0.667 就是三步态 0.833就是六步态
        self.cmdGait["SwapHigh"] = 100.0  # type: ignore     # Swing height (mm) 摆动高度
        self.cmdGait["LegNum"] = 0  # type: ignore    # Force control mode # 单腿运动的时候控制这个，这个先给0
        self.cmdGait["ForceMode"] = 0 # type: ignore    # 无力控是0
        self.cmdGait["Res"] = 0  # type: ignore        # 0 无意义
        self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore 
        
        # Set pose parameters
        self.cmdPose["X"] = min(400, (aim_pose.pose.position.x - cur_pose.position.x) * 1000) # type: ignore    # Convert to mm
        self.cmdPose["Y"] = min(200, (aim_pose.pose.position.y - cur_pose.position.y) * 1000)  # type: ignore   
        self.cmdPose["Z"] = 0.0  # type: ignore   
        print(f"[move to] cmdPose set is: x: {(aim_pose.pose.position.x - cur_pose.position.x) * 1000}, y: {(aim_pose.pose.position.y - cur_pose.position.y) * 1000}")
        
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
    

        
    def follow_trajectory(self, trajectory: Path) -> bool:
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
            print(f"[follow_traj]cur_step= {cur_step} aim_pose x= {aim_pose.position.x}"
                  f"y= {aim_pose.position.y} z= {aim_pose.position.z}")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            if cur_beifu_Cmd["FG"]==CtrlCmd.IDLE:
                # 拿一下机器人当前的位置
                cur_pose:Pose = self.get_current_pose()
                print(f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z}")
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit:
                    # 说明已经到达了当前点
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.position.x}, y: {aim_pose.position.y}, z: {aim_pose.position.z}")
                    cur_step += 1
                else:
                    print(f"[follow_traj]begin move to aim_pose x: {aim_pose.position.x}, y: {aim_pose.position.y}, z: {aim_pose.position.z}")
                    self.move_to_pos(cur_pose, aim_pose)
            
        return True
    
    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
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
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYawDiff2d(aim_pose, cur_pose)) <= self.admit_angle_limit:
                    # 说明已经到达了当前点
                    print(f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    print(f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    
                    self.move_to_pos(cur_pose, aim_pose)
                    # 虚拟地将移动后的期望位置更新为机器人的当前位置
                    self.current_pose.position.x = aim_pose.pose.position.x
                    self.current_pose.position.y = aim_pose.pose.position.y
                    self.current_pose.position.z = aim_pose.pose.position.z
            
        return True
    
    
    
    
    
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
    
    def cleanup(self):
        """Cleanup PLC connection"""
        if self.plc_connected and self.plc:
            try:
                self.symbol_State.write(State.DISENABLE) # type: ignore
                self.plc.close()
                rospy.loginfo("PLC connection closed")
            except Exception as e:
                rospy.logerr(f"Error closing PLC connection: {str(e)}")


def main():
    """Main function to run hexapod interface"""
    # Initialize ROS node first
    rospy.init_node('hexapod201_interface', anonymous=True)
    
    interface_type = rospy.get_param('~interface_type', None)
    use_dummy = rospy.get_param('~dummy', False)
    plc_ip = rospy.get_param('~plc_ip', '5.157.100.214.1.1')
    node_name = rospy.get_param('~node_name', 'hexapod201_interface')
    
    try:
        # Prefer interface_type param if set
        if interface_type is not None:
            if isinstance(interface_type, str) and interface_type.lower() == 'dummy':
                interface = DummyHexapod201Interface(str(node_name))
                print("DummyHexapod201Interface")
            else:
                interface = Hexapod201Interface(str(node_name), str(plc_ip))
                print("Hexapod201Interface")
        else:
            if use_dummy:
                interface = DummyHexapod201Interface(str(node_name))
                print("DummyHexapod201Interface")
            else:
                interface = Hexapod201Interface(str(node_name), str(plc_ip))
                print("Hexapod201Interface")
        
        rospy.loginfo("Hexapod interface started")
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down hexapod interface")
        if hasattr(interface, 'cleanup'):
            interface.cleanup()
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")

def test_interface():
    rospy.init_node('test_hexapod201_interface', anonymous=True)
    interface = Hexapod201Interface(node_name="hexapod201_interface", plc_ip="5.157.100.214.1.1")
    pose = Pose()
    pose.position.x = 0.1
    pose.position.y = 0.0
    pose.position.z = 0.0
    quat = quaternion_from_euler(0.0, 0.0, 0.0)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    interface.move_to_pose(pose)
    rospy.sleep(5)  # Wait for movement to complete
    current_pose = interface.get_current_pose()
    rospy.loginfo(f"Current pose after movement: {current_pose}")

if __name__ == "__main__":
    # main()
    test_interface()
    # time.sleep(10.0)
    # rospy.init_node("rospy_interface")
    
    # trajectory: List[Pose]
