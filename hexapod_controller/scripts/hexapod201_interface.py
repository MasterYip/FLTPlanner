#!/usr/bin/env python3

import rospy
import numpy as np
import pyads
import time
import threading
from typing import Dict, Any, Optional, Tuple
from enum import IntEnum
from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import copy
# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle
# Import FootState message
from legged_traj_plan.msg import FootState


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

# Additional structure for free gait foothold definition
stXYZ_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("SF", pyads.PLCTYPE_DINT, 1),
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
        
        # Cached pose from pose command - will be used when foot command arrives
        self.cached_target_pose = None
        self.has_cached_pose = False
        
        self.is_moving = False
        self.last_cmd_time = rospy.Time.now()
        self.cmd_vel_integration = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        
        # ROS node should be initialized before creating this class
        
        # Publishers and subscribers
        self.pose_pub = rospy.Publisher('/hexapod/current_pose', PoseStamped, queue_size=10)
        # self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/pose_cmd', PoseStamped, self.pose_cmd_callback)
        self.foot_cmd_sub = rospy.Subscriber('/hexapod/foot_cmd', FootState, self.foot_cmd_callback)
        
        # Timer for publishing current pose
        self.pose_timer = rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        self.dt = 1.0
        # Visualization
        self.visualizer = ROSVisualizer("odom", "hexapod_visualization")
        
        # Free gait parameters
        self.foot_positions = np.zeros((6, 3))  # Current foot positions [x, y, z] in mm
        self.target_foot_positions = np.zeros((6, 3))  # Target foot positions [x, y, z] in mm
        self.foot_support_flags = np.zeros(6, dtype=int)  # 0=support, 1=swing
        self.default_foot_positions = np.array([
            [660, 996.2, -405],   # Foot 1
            [0, 1251.2, -405],    # Foot 2
            [-660, 996.2, -405],  # Foot 3
            [660, -996.2, -405],  # Foot 4
            [0, -1251.2, -405],   # Foot 5
            [-660, -996.2, -405]  # Foot 6
        ])
        self.foot_positions = self.default_foot_positions.copy()
        self.target_foot_positions = self.default_foot_positions.copy()
        
        rospy.loginfo(f"{node_name} initialized")
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands by integrating to get target pose"""
        # current_time = rospy.Time.now()
        # dt = (current_time - self.last_cmd_time).to_sec()
        # self.last_cmd_time = current_time
        dt = self.dt  # Use fixed dt for simplicity
        # Integrate velocity to get position change
        linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        # Simple Euler integration
        self.cmd_vel_integration[:3] = linear_vel * dt
        self.cmd_vel_integration[3:] = angular_vel * dt
        
        # Create target pose from current pose + integration
        self.target_pose.position.x = self.current_pose.position.x + self.cmd_vel_integration[0]
        self.target_pose.position.y = self.current_pose.position.y + self.cmd_vel_integration[1]
        self.target_pose.position.z = self.current_pose.position.z + self.cmd_vel_integration[2]
        
        # Convert Euler angles to quaternion
        # BUG: this is not correct
        roll, pitch, yaw = self.cmd_vel_integration[3], self.cmd_vel_integration[4], self.cmd_vel_integration[5]
        quat = quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.orientation.w = quat[3]
        self.target_pose.orientation.x = quat[0]
        self.target_pose.orientation.y = quat[1]
        self.target_pose.orientation.z = quat[2]

        # Execute movement
        self.move_to_pose(self.target_pose)
    
    def pose_cmd_callback(self, msg: PoseStamped):
        """Cache pose command - movement will be triggered by foot state callback"""
        self.cached_target_pose = msg.pose
        self.has_cached_pose = True
        rospy.loginfo("Cached target pose from pose command")
    
    def foot_cmd_callback(self, msg: FootState):
        """Handle foot command and execute movement using cached pose + foot positions"""
        try:
            # Extract foot positions from the FootState message
            foot_positions = np.zeros((6, 3))
            foot_velocities = np.zeros((6, 3))
            foot_efforts = np.zeros((6, 3))
            contact_states = np.zeros(6, dtype=bool)
            
            num_feet = min(len(msg.position), 6)
            
            for i in range(num_feet):
                # Position (convert from m to mm if needed, assuming message is in meters)
                foot_positions[i, 0] = msg.position[i].x * 1000.0  # Convert m to mm
                foot_positions[i, 1] = msg.position[i].y * 1000.0
                foot_positions[i, 2] = msg.position[i].z * 1000.0
                
                # Velocity (if available)
                if i < len(msg.velocity):
                    foot_velocities[i, 0] = msg.velocity[i].x * 1000.0
                    foot_velocities[i, 1] = msg.velocity[i].y * 1000.0
                    foot_velocities[i, 2] = msg.velocity[i].z * 1000.0
                
                # Effort (if available)
                if i < len(msg.effort):
                    foot_efforts[i, 0] = msg.effort[i].x
                    foot_efforts[i, 1] = msg.effort[i].y
                    foot_efforts[i, 2] = msg.effort[i].z
                
                # Contact state (if available)
                if i < len(msg.contact):
                    contact_states[i] = msg.contact[i]
                else:
                    contact_states[i] = True  # Default to contact
            
            # Determine foot support flags based on contact state
            # FootState.contact: true = in contact (support), false = not in contact (swing)
            foot_flags = np.zeros(6, dtype=int)
            for i in range(num_feet):
                if contact_states[i]:
                    foot_flags[i] = 0  # Support
                else:
                    foot_flags[i] = 1  # Swing
                
                # Alternative: also consider velocity magnitude for swing detection
                vel_magnitude = np.linalg.norm(foot_velocities[i])
                if vel_magnitude > 10.0:  # mm/s threshold
                    foot_flags[i] = 1  # Swing
            
            # Use cached target pose if available, otherwise use current pose
            target_pose = self.cached_target_pose if self.has_cached_pose else self.current_pose
            
            if self.has_cached_pose:
                # Clear cached pose after using it
                self.has_cached_pose = False
                rospy.loginfo(f"Received FootState for {num_feet} feet, using cached target pose")
            else:
                rospy.loginfo(f"Received FootState for {num_feet} feet, no cached pose - using current pose")
            
            # Execute coordinated movement with target pose and foot positions
            self.move_to_pose_with_feet(target_pose, foot_positions, foot_flags)
            
        except Exception as e:
            rospy.logerr(f"Error processing FootState: {str(e)}")
    
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
        heading_vec = np.array([np.cos(rpy[2]), np.sin(rpy[2]), 0.0])  # Heading direction in XY plane
        # Box size (hexapod body dimensions)
        box_size = [0.6, 0.3, 0.2]  # 30cm cube
        self.visualizer.vis_cube(position, quat, VisStyle(1.0, 0.45, 0.0, 1.0, box_size[0], box_size[1], box_size[2]))
        self.visualizer.vis_arrow(position, position + heading_vec * 0.4)
    
    @abstractmethod
    def move_to_pose(self, target_pose: Pose) -> bool:
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
    
    @abstractmethod
    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod using free gait with custom foothold definitions
        
        Args:
            body_motion: [x, y, z, roll, pitch, yaw] body motion in mm and radians
            foot_positions: (6, 3) array of foot positions [x, y, z] in mm (robot body frame)
            foot_flags: (6,) array of foot flags (0=support, 1=swing)
        """
        pass

    @abstractmethod
    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod to target pose with specific foot positions
        
        Args:
            target_pose: Target body pose
            foot_positions: (6, 3) array of foot positions [x, y, z] in mm (robot body frame)
            foot_flags: (6,) array of foot flags (0=support, 1=swing)
        """
        pass
    
    def set_default_foot_positions(self, positions: np.ndarray):
        """Set default foot positions for support stance"""
        if positions.shape == (6, 3):
            self.default_foot_positions = positions.copy()
            
    def get_foot_positions(self) -> np.ndarray:
        """Get current foot positions"""
        return self.foot_positions.copy()
    
    def get_target_foot_positions(self) -> np.ndarray:
        """Get target foot positions"""
        return self.target_foot_positions.copy()
    
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
    
    def __init__(self, node_name: str = "dummy_hexapod201_interface"):
        super().__init__(node_name)
        self.movement_speed = 1.0  # m/s
        self.rotation_speed = 10.0  # rad/s
        self.movement_thread = None
        self.movement_lock = threading.Lock()
        
        # Free gait specific parameters
        self.foot_start_positions = np.zeros((6, 3))
        self.swing_phase = np.zeros(6)  # 0-1 swing phase for each foot
        self.swing_height = 80.0  # mm
        self.swing_duration = 1.5  # seconds
        
        # Cache world positions for support feet (they should not move in world frame)
        self.support_foot_world_positions = np.zeros((6, 3))  # World frame positions for support feet
        
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
            if angle_diff[2] > np.pi:
                angle_diff[2] -= 2 * np.pi
            elif angle_diff[2] < -np.pi:
                angle_diff[2] += 2 * np.pi
            
            # Calculate movement time
            pos_distance = np.linalg.norm(pos_diff)
            angle_distance = np.linalg.norm(angle_diff)
            
            pos_time = pos_distance / self.movement_speed if pos_distance > 0 else 0
            angle_time = angle_distance / self.rotation_speed if angle_distance > 0 else 0
            total_time = max(pos_time, angle_time)
            
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
                    
                    if progress >= 1.0:
                        break
                    
                    rate.sleep()
            
            rospy.loginfo("Dummy movement completed")
            
        except Exception as e:
            rospy.logerr(f"Error in dummy movement: {str(e)}")
        finally:
            with self.movement_lock:
                self.is_moving = False

    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod using free gait with foot visualization"""
        with self.movement_lock:
            if self.is_moving:
                rospy.logwarn("Free gait movement already in progress")
                return False
            
            self.is_moving = True
            
            # Store initial foot positions for swing interpolation
            self.foot_start_positions = self.foot_positions.copy()
            self.target_foot_positions = foot_positions.copy()
            self.foot_support_flags = foot_flags.copy()
            
            # Store body motion
            self.target_body_motion = body_motion.copy()
            
            # Start free gait movement in separate thread
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join()
            
            self.movement_thread = threading.Thread(target=self._execute_free_gait_movement)
            self.movement_thread.start()
            
            return True
    
    def _execute_free_gait_movement(self):
        """Execute free gait movement with foot interpolation"""
        try:
            start_time = rospy.Time.now()
            rate = rospy.Rate(50)  # 50 Hz update rate
            
            # Store initial body pose
            start_pose = copy.deepcopy(self.current_pose)
            
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
                elapsed = (current_time - start_time).to_sec()
                progress = min(elapsed / self.swing_duration, 1.0)
                
                # Update body pose
                self.current_pose.position.x = start_pose.position.x + (self.target_body_motion[0] / 1000.0) * progress  # Convert mm to m
                self.current_pose.position.y = start_pose.position.y + (self.target_body_motion[1] / 1000.0) * progress
                self.current_pose.position.z = start_pose.position.z + (self.target_body_motion[2] / 1000.0) * progress
                
                # Update orientation
                start_euler = euler_from_quaternion([
                    start_pose.orientation.x,
                    start_pose.orientation.y,
                    start_pose.orientation.z,
                    start_pose.orientation.w,
                ])
                
                current_euler = start_euler + self.target_body_motion[3:6] * progress
                quat = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
                self.current_pose.orientation.w = quat[3]
                self.current_pose.orientation.x = quat[0]
                self.current_pose.orientation.y = quat[1]
                self.current_pose.orientation.z = quat[2]
                
                # Update foot positions
                for i in range(6):
                    if self.foot_support_flags[i] == 1:  # Swing foot
                        # Hermite interpolation for swing phase
                        self.foot_positions[i] = self._hermite_interpolate_foot(i, progress)
                    else:  # Support foot
                        # Linear interpolation for support phase
                        self.foot_positions[i] = self._linear_interpolate_foot(i, progress)
                
                # Visualize feet
                self._visualize_feet()
                
                if progress >= 1.0:
                    break
                
                rate.sleep()
            
            rospy.loginfo("Free gait movement completed")
            
        except Exception as e:
            rospy.logerr(f"Error in free gait movement: {str(e)}")
        finally:
            with self.movement_lock:
                self.is_moving = False
    
    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod to target pose with specific foot positions"""
        with self.movement_lock:
            if self.is_moving:
                rospy.logwarn("Coordinated movement already in progress")
                return False
            
            self.is_moving = True
            self.target_pose = target_pose
            
            # Store foot data for coordinated movement
            self.foot_start_positions = self.foot_positions.copy()
            self.target_foot_positions = foot_positions.copy()
            self.foot_support_flags = foot_flags.copy()
            
            # Cache world positions for support feet at the start of movement
            body_pos = np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z
            ])
            
            body_quat = np.array([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ])
            
            # Transform support feet to world frame and cache their positions
            from tf.transformations import quaternion_matrix
            transform_matrix = quaternion_matrix(body_quat)
            rotation_matrix = transform_matrix[:3, :3]
            
            for i in range(6):
                if foot_flags[i] == 0:  # Support foot
                    foot_pos_body = self.foot_positions[i] / 1000.0  # Convert mm to m
                    foot_pos_world = body_pos + rotation_matrix.dot(foot_pos_body)
                    self.support_foot_world_positions[i] = foot_pos_world * 1000.0  # Store in mm
            
            # Start coordinated movement in separate thread
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join()
            
            self.movement_thread = threading.Thread(target=self._execute_coordinated_movement)
            self.movement_thread.start()
            
            return True
    
    def _execute_coordinated_movement(self):
        """Execute coordinated body pose and foot position movement"""
        try:
            start_time = rospy.Time.now()
            rate = rospy.Rate(50)  # 50 Hz update rate
            
            # Store initial body pose
            start_pose = copy.deepcopy(self.current_pose)
            target_pose = self.target_pose
            
            # Calculate body motion deltas
            pos_diff = np.array([
                target_pose.position.x - start_pose.position.x,
                target_pose.position.y - start_pose.position.y,
                target_pose.position.z - start_pose.position.z
            ])
            
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
            
            # Normalize yaw angle difference
            if angle_diff[2] > np.pi:
                angle_diff[2] -= 2 * np.pi
            elif angle_diff[2] < -np.pi:
                angle_diff[2] += 2 * np.pi
            
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
                elapsed = (current_time - start_time).to_sec()
                progress = min(elapsed / self.swing_duration, 1.0)
                
                # Update body pose
                self.current_pose.position.x = start_pose.position.x + pos_diff[0] * progress
                self.current_pose.position.y = start_pose.position.y + pos_diff[1] * progress
                self.current_pose.position.z = start_pose.position.z + pos_diff[2] * progress
                
                # Update orientation
                current_euler = start_euler + angle_diff * progress
                quat = quaternion_from_euler(current_euler[0], current_euler[1], current_euler[2])
                self.current_pose.orientation.w = quat[3]
                self.current_pose.orientation.x = quat[0]
                self.current_pose.orientation.y = quat[1]
                self.current_pose.orientation.z = quat[2]
                
                # Update foot positions
                for i in range(6):
                    if self.foot_support_flags[i] == 1:  # Swing foot
                        # Hermite interpolation for swing phase
                        self.foot_positions[i] = self._hermite_interpolate_foot(i, progress)
                    else:  # Support foot - keep stationary in world frame
                        # Transform cached world position back to current body frame
                        world_pos = self.support_foot_world_positions[i] / 1000.0  # Convert mm to m
                        
                        # Get current body transform
                        current_body_pos = np.array([
                            self.current_pose.position.x,
                            self.current_pose.position.y,
                            self.current_pose.position.z
                        ])
                        
                        current_body_quat = np.array([
                            self.current_pose.orientation.x,
                            self.current_pose.orientation.y,
                            self.current_pose.orientation.z,
                            self.current_pose.orientation.w,
                        ])
                        
                        # Transform world position to current body frame
                        from tf.transformations import quaternion_matrix
                        transform_matrix = quaternion_matrix(current_body_quat)
                        rotation_matrix = transform_matrix[:3, :3]
                        
                        # World to body transformation: body_pos = R^T * (world_pos - body_pos)
                        body_relative_pos = world_pos - current_body_pos
                        foot_pos_body = rotation_matrix.T.dot(body_relative_pos)
                        
                        self.foot_positions[i] = foot_pos_body * 1000.0  # Convert back to mm
                
                # Visualize feet
                self._visualize_feet()
                
                if progress >= 1.0:
                    break
                
                rate.sleep()
            
            rospy.loginfo("Coordinated movement completed")
            
        except Exception as e:
            rospy.logerr(f"Error in coordinated movement: {str(e)}")
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

    def _hermite_interpolate_foot(self, foot_idx: int, t: float) -> np.ndarray:
        """Hermite interpolation for swing phase foot trajectory"""
        start_pos = self.foot_start_positions[foot_idx]
        end_pos = self.target_foot_positions[foot_idx]
        
        # Hermite basis functions
        h1 = 2*t**3 - 3*t**2 + 1
        h2 = -2*t**3 + 3*t**2
        h3 = t**3 - 2*t**2 + t
        h4 = t**3 - t**2
        
        # Start and end velocities (can be adjusted)
        start_vel = np.array([0.0, 0.0, 0.5])
        end_vel = np.array([0.0, 0.0, -0.5])
        
        # Base trajectory (without height)
        base_pos = h1 * start_pos + h2 * end_pos + h3 * start_vel + h4 * end_vel
        
        # Add swing height using a parabolic trajectory
        height_factor = 4 * t * (1 - t)  # Parabolic curve, max at t=0.5
        swing_offset = np.array([0.0, 0.0, self.swing_height * height_factor])
        
        return base_pos + swing_offset
    
    def _linear_interpolate_foot(self, foot_idx: int, t: float) -> np.ndarray:
        """Linear interpolation for support phase foot trajectory"""
        start_pos = self.foot_start_positions[foot_idx]
        end_pos = self.target_foot_positions[foot_idx]
        
        return start_pos + (end_pos - start_pos) * t
    
    def _visualize_feet(self):
        """Visualize feet as small spheres in RViz with lines connecting to body center"""
        # Get current body position and orientation for coordinate transformation
        body_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        body_quat = np.array([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ])
        
        # Convert body quaternion to rotation matrix for coordinate transformation
        from tf.transformations import quaternion_matrix
        transform_matrix = quaternion_matrix(body_quat)
        rotation_matrix = transform_matrix[:3, :3]
        
        foot_colors = [
            [1.0, 0.0, 0.0, 1.0],  # Red
            [0.0, 1.0, 0.0, 1.0],  # Green
            [0.0, 0.0, 1.0, 1.0],  # Blue
            [1.0, 1.0, 0.0, 1.0],  # Yellow
            [1.0, 0.0, 1.0, 1.0],  # Magenta
            [0.0, 1.0, 1.0, 1.0],  # Cyan
        ]
        
        for i in range(6):
            # Transform foot position from body frame to world frame
            foot_pos_body = self.foot_positions[i] / 1000.0  # Convert mm to m
            foot_pos_world = body_pos + rotation_matrix.dot(foot_pos_body)
            
            # Choose sphere size based on support/swing state
            sphere_size = 0.03 if self.foot_support_flags[i] == 0 else 0.02  # Larger for support
            
            # Create sphere style
            style = VisStyle(
                foot_colors[i][0], foot_colors[i][1], foot_colors[i][2], foot_colors[i][3],
                sphere_size, sphere_size, sphere_size
            )
            
            # Visualize foot sphere
            self.visualizer.vis_sphere(foot_pos_world, sphere_size, style)
            
            # Visualize line connecting body center to foot
            line_style = VisStyle(
                foot_colors[i][0], foot_colors[i][1], foot_colors[i][2], 0.6,  # Semi-transparent
                0.005, 0.005, 0.005  # Thin line
            )
            self.visualizer.vis_arrow(body_pos, foot_pos_world, line_style)


class Hexapod201Interface(Hexapod201BaseInterface):
    """Real hexapod interface using PLC communication"""
    
    def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "192.168.1.115.1.1"):
        super().__init__(node_name)
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
        
        # Connect to PLC and CPP
        self._connect_plc()
        
        rospy.loginfo("Real hexapod interface initialized")
    
    def _connect_plc(self):
        """Establish PLC and CPP connections"""
        # try:

        # PLC connection
        self.plc = pyads.Connection(self.plc_ip, pyads.PORT_TC3PLC1)
        self.plc.open()
        
        # CPP connection for free gait
        self.cpp = pyads.Connection(self.plc_ip, 351)
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
        self.symbol_PTActPos = self.plc.get_symbol('MAIN.PTActPos', structure_def=stPose_def)

        # Initialize CPP symbols for free gait
        self.symbol_ReqPTCmd = self.cpp.get_symbol('CPP.Inputs.ReqPTCmd', structure_def=stPose_def)
        self.symbol_ReqFlag = self.cpp.get_symbol('CPP.Inputs.ReqFlag')
        
        # Enable auto-update for feedback
        self.symbol_QState.auto_update = True
        self.symbol_ReqFlag.auto_update = True
        self.symbol_PTActPos.auto_update = True
        
        self.plc_connected = True
        self.cpp_connected = True
        rospy.loginfo("PLC and CPP connections established")
            
        # except Exception as e:
        #     rospy.logerr(f"PLC/CPP connection failed: {str(e)}")
        #     self.plc_connected = False
        #     self.cpp_connected = False
    
    def _enable_plc(self):
        """Enable PLC for movement"""
        if not self.plc_connected:
            return False
        
        try:
            self.symbol_State.write(State.ENABLE)
            
            # Wait for enable
            timeout = 10.0
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                if self.symbol_QState.value == State.FEEDMOV:
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
                self.cmdTime = self.symbol_Cmd_Time.read()
            if self.cmdGait is None:
                self.cmdGait = self.symbol_Cmd_Gait.read()
            if self.cmdPose is None:
                self.cmdPose = self.symbol_Cmd_Pose.read()
            
            return True
        except Exception as e:
            rospy.logerr(f"Failed to read PLC parameters: {str(e)}")
            return False
    
    def _update_current_pose_from_plc(self):
        """Update current pose from PLC feedback"""
        if not self.plc_connected:
            return
        
        try:
            act_pos = self.symbol_PTActPos.read()
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
        
        # try:
        # Enable PLC
        if not self._enable_plc():
            return False
        
        # Read current parameters
        if not self._read_plc_parameters():
            return False
        
        # Set movement parameters
        self.cmdTime["TA"] = 0.5  # Acceleration time
        self.cmdTime["TM"] = 2.5  # Movement time
        self.cmdTime["TD"] = 0.0  # Deceleration overlap
        self.cmdTime["TZ"] = 0.0  # Z advance time
        self.symbol_Cmd_Time.write(self.cmdTime)
        
        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # Synchronous gait
        self.cmdGait["GaitDF"] = 0.5  # Duty factor
        self.cmdGait["SwapHigh"] = 80.0  # Swing height (mm)
        self.cmdGait["LegNum"] = 0  
        self.cmdGait["ForceMode"] = 0 # Force control mode
        self.cmdGait["Res"] = 0
        self.symbol_Cmd_Gait.write(self.cmdGait)
        
        # Set pose parameters
        self.cmdPose["X"] = target_pose.position.x * 1000  # Convert to mm
        self.cmdPose["Y"] = target_pose.position.y * 1000
        self.cmdPose["Z"] = target_pose.position.z * 1000
        
        # Convert quaternion to Euler angles
        euler = euler_from_quaternion([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        self.cmdPose["Roll"] = euler[0]
        self.cmdPose["Pitch"] = euler[1]
        self.cmdPose["Yaw"] = euler[2]
        
        self.cmdPose["FG"] = 0  # Movement mode
        self.cmdPose["Res"] = 0
        self.symbol_Cmd_Pose.write(self.cmdPose)
        
        # Start movement
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)
        
        rospy.loginfo(f"Started movement to pose: {target_pose.position}")
        return True
            
        # except Exception as e:
        #     rospy.logerr(f"Movement failed: {str(e)}")
        #     return False
    
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
            self.cmdTime["TA"] = 0.5  # Acceleration time
            self.cmdTime["TM"] = 1.5  # Swing time
            self.cmdTime["TD"] = 0.0  # Support overlap time
            self.cmdTime["TZ"] = 0.0  # Z advance time
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
            else:
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
            for i in range(6):
                x_key = f"X{i+1}"
                y_key = f"Y{i+1}"
                z_key = f"Z{i+1}"
                sf_key = f"SF{i+1}"
                
                self.ReqPTCmd[x_key] = foot_positions[i, 0]  # mm
                self.ReqPTCmd[y_key] = foot_positions[i, 1]  # mm
                self.ReqPTCmd[z_key] = foot_positions[i, 2]  # mm
                self.ReqPTCmd[sf_key] = foot_flags[i]  # 0=support, 1=swing
            
            # Send command to CPP
            self.symbol_ReqPTCmd.write(self.ReqPTCmd)
            self.symbol_ReqFlag.write(2)  # Start movement
            
            # Update internal foot positions
            self.foot_positions = foot_positions.copy()
            self.target_foot_positions = foot_positions.copy()
            self.foot_support_flags = foot_flags.copy()
            
            rospy.loginfo(f"Started free gait movement with body motion: {body_motion}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Free gait movement failed: {str(e)}")
            return False
    
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
            if angle_diff[2] > np.pi:
                angle_diff[2] -= 2 * np.pi
            elif angle_diff[2] < -np.pi:
                angle_diff[2] += 2 * np.pi
            
            # Create body motion array
            body_motion = np.concatenate([pos_diff, angle_diff])
            
            # Use the existing free gait method with calculated body motion
            success = self.move_free_gait(body_motion, foot_positions, foot_flags)
            
            if success:
                # Update current pose to target pose
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
                self.cmdTime["TA"] = kwargs['TA']
            if 'TM' in kwargs:
                self.cmdTime["TM"] = kwargs['TM']
            if 'TD' in kwargs:
                self.cmdTime["TD"] = kwargs['TD']
            if 'TZ' in kwargs:
                self.cmdTime["TZ"] = kwargs['TZ']
            
            # Gait parameters
            if 'GaitMode' in kwargs:
                self.cmdGait["GaitMode"] = kwargs['GaitMode']
            if 'GaitDF' in kwargs:
                self.cmdGait["GaitDF"] = kwargs['GaitDF']
            if 'SwapHigh' in kwargs:
                self.cmdGait["SwapHigh"] = kwargs['SwapHigh']
            if 'ForceMode' in kwargs:
                self.cmdGait["ForceMode"] = kwargs['ForceMode']
            
            # Write parameters to PLC
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
        """Stop current movement"""
        if not self.plc_connected:
            return False
        
        try:
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)
            rospy.loginfo("Movement stopped")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to stop movement: {str(e)}")
            return False
    
    def cleanup(self):
        """Cleanup PLC and CPP connections"""
        if self.plc_connected and self.plc:
            try:
                self.symbol_State.write(State.DISENABLE)
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


def main():
    """Main function to run hexapod interface"""
    # Initialize ROS node first
    rospy.init_node('hexapod201_interface', anonymous=True)
    
    # Get parameters from ROS parameter server
    interface_type = rospy.get_param('~interface_type', None)
    use_dummy = rospy.get_param('~dummy', False)
    plc_ip = rospy.get_param('~plc_ip', '192.168.1.115.1.1')
    node_name = rospy.get_param('~node_name', 'hexapod201_interface')
    
    try:
        # Prefer interface_type param if set
        if interface_type is not None:
            if interface_type.lower() == 'dummy':
                interface = DummyHexapod201Interface(node_name)
            else:
                interface = Hexapod201Interface(node_name, plc_ip)
        else:
            if use_dummy:
                interface = DummyHexapod201Interface(node_name)
            else:
                interface = Hexapod201Interface(node_name, plc_ip)
        
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
    interface = Hexapod201Interface(node_name="hexapod201_interface", plc_ip="192.168.1.115.1.1")
    pose = Pose()
    pose.position.x = 0.2
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
    main()
    # test_interface()