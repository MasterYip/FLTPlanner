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
import geometry_msgs.msg  # Add this import for Point and Vector3
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path
import copy
# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle  # pyright: ignore[reportAttributeAccessIssue]
# Import FootState message
from legged_traj_plan.msg import FootState
import tf2_ros
# Hexapod201 Default Configuration Constants
HEXAPOD201_DEFAULT_FOOT_POSITIONS = np.array([
    [660, -996.2, -405],  # Foot201 4 - FootElAir 0
    [0, -1251.2, -405],   # Foot201 5 - FootElAir 1
    [-660, -996.2, -405],  # Foot201 6 - FootElAir 2
    [660, 996.2, -405],   # Foot201 1 - FootElAir 3
    [0, 1251.2, -405],    # Foot201 2 - FootElAir 4
    [-660, 996.2, -405],  # Foot201 3 - FootElAir 5
])

FOOT_REMAP = [3, 4, 5, 0, 1, 2]  # Remap from FootElAir to Foot201

HEXAPOD201_DEFAULT_BODY_HEIGHT = 0.405  # meters
HEXAPOD201_DEFAULT_INIT_POSE = {
    'position': {'x': 0.0, 'y': 0.0, 'z': HEXAPOD201_DEFAULT_BODY_HEIGHT},
    'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
}


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
# Additional structure for free gait foothold definition
stJointPos_def = (
    ("A1", pyads.PLCTYPE_REAL, 1),  # 左前
    ("B1", pyads.PLCTYPE_REAL, 1),
    ("C1", pyads.PLCTYPE_REAL, 1),
    ("F1", pyads.PLCTYPE_DINT, 1),  # F在别的变量里代表是摆动还是支撑，在这里是保留的，是为了字节对齐还有一些其他的东西

    ("A2", pyads.PLCTYPE_REAL, 1),  # 左中
    ("B2", pyads.PLCTYPE_REAL, 1),
    ("C2", pyads.PLCTYPE_REAL, 1),
    ("F2", pyads.PLCTYPE_DINT, 1),

    ("A3", pyads.PLCTYPE_REAL, 1),  # 左后
    ("B3", pyads.PLCTYPE_REAL, 1),
    ("C3", pyads.PLCTYPE_REAL, 1),
    ("F3", pyads.PLCTYPE_DINT, 1),

    ("A4", pyads.PLCTYPE_REAL, 1),
    ("B4", pyads.PLCTYPE_REAL, 1),
    ("C4", pyads.PLCTYPE_REAL, 1),
    ("F4", pyads.PLCTYPE_DINT, 1),

    ("A5", pyads.PLCTYPE_REAL, 1),
    ("B5", pyads.PLCTYPE_REAL, 1),
    ("C5", pyads.PLCTYPE_REAL, 1),
    ("F5", pyads.PLCTYPE_DINT, 1),

    ("A6", pyads.PLCTYPE_REAL, 1),
    ("B6", pyads.PLCTYPE_REAL, 1),
    ("C6", pyads.PLCTYPE_REAL, 1),
    ("F6", pyads.PLCTYPE_DINT, 1),

)


class Hexapod201BaseInterface(ABC):
    """Base interface for hexapod control"""

    def __init__(self, node_name: str = "hexapod201_interface"):
        self.node_name = node_name

        # Initialize poses using the extracted constants
        self.current_pose = Pose()
        self.current_pose.position.x = HEXAPOD201_DEFAULT_INIT_POSE['position']['x']
        self.current_pose.position.y = HEXAPOD201_DEFAULT_INIT_POSE['position']['y']
        self.current_pose.position.z = HEXAPOD201_DEFAULT_INIT_POSE['position']['z']
        self.current_pose.orientation.w = HEXAPOD201_DEFAULT_INIT_POSE['orientation']['w']
        self.current_pose.orientation.x = HEXAPOD201_DEFAULT_INIT_POSE['orientation']['x']
        self.current_pose.orientation.y = HEXAPOD201_DEFAULT_INIT_POSE['orientation']['y']
        self.current_pose.orientation.z = HEXAPOD201_DEFAULT_INIT_POSE['orientation']['z']

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
        self.foot_state_pub = rospy.Publisher('/hexapod/foot_state', FootState, queue_size=10)
        # self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/pose_cmd', PoseStamped, self.pose_cmd_callback)
        self.foot_cmd_sub = rospy.Subscriber('/hexapod/foot_cmd', FootState, self.foot_cmd_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/path_cmd', Path, self.follow_virtual_trajectory)

        # Timer for publishing current pose and foot state
        self.pose_timer = rospy.Timer(rospy.Duration(0, int(1e8)), self.publish_feedback)
        self.dt = 1.0
        # Visualization
        self.visualizer = ROSVisualizer("world", "hexapod_visualization")

        # Free gait parameters
        self.foot_positions = np.zeros((6, 3))  # Current foot positions [x, y, z] in mm
        self.target_foot_positions = np.zeros((6, 3))  # Target foot positions [x, y, z] in mm
        self.foot_support_flags = np.zeros(6, dtype=int)  # 0=support, 1=swing
        self.default_foot_positions = HEXAPOD201_DEFAULT_FOOT_POSITIONS.copy()
        # self.foot_positions = self.default_foot_positions.copy()
        self.target_foot_positions = self.default_foot_positions.copy()

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

    # 发布当前位置, 将当前位置在odom坐标系下发布, 并可视化一个长方体形状的六足机体

    def publish_current_pose(self, event):
        """Publish current pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)

        # Visualize hexapod body as a box
        self._vis_body()

    def _vis_body(self):
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
        rpy = euler_from_quaternion(quat)
        heading_vec = np.array([math.cos(rpy[2]), math.sin(rpy[2]), 0.0])  # Heading direction in XY plane
        # Box size (hexapod body dimensions)
        box_size = [0.6, 0.3, 0.2]  # 30cm cube
        self.visualizer.vis_cube(position, quat, VisStyle(1.0, 0.45, 0.0, 1.0, box_size[0], box_size[1], box_size[2]))
        self.visualizer.vis_arrow(position, position + heading_vec * 0.4)

    def publish_feedback(self, event):
        """Publish current pose and foot state feedback"""
        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)

        # Publish current foot state
        foot_state_msg = FootState()
        foot_state_msg.header.stamp = rospy.Time.now()
        foot_state_msg.header.frame_id = "base_link"

        # Add foot names and data
        for i in range(6):
            foot_state_msg.name.append(f"foot_{i}")

            # Position (convert from mm to m for ROS standard)
            pos = geometry_msgs.msg.Point()
            pos.x = self.foot_positions[i, 0] / 1000.0
            pos.y = self.foot_positions[i, 1] / 1000.0
            pos.z = self.foot_positions[i, 2] / 1000.0
            foot_state_msg.position.append(pos)

            # Velocity (zeros for now)
            vel = geometry_msgs.msg.Vector3()
            vel.x = vel.y = vel.z = 0.0
            foot_state_msg.velocity.append(vel)

            # Effort (zeros for now)
            effort = geometry_msgs.msg.Vector3()
            effort.x = effort.y = effort.z = 0.0
            foot_state_msg.effort.append(effort)

            # Contact state (assume all feet in contact by default)
            contact_state = self.foot_support_flags[i] == 0 if len(self.foot_support_flags) > i else True
            foot_state_msg.contact.append(contact_state)

        self.foot_state_pub.publish(foot_state_msg)

        # Also call visualization (for dummy interface)
        if hasattr(self, '_vis_body'):
            self._vis_body()

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

        # Free gait specific parameters
        self.foot_start_positions = np.zeros((6, 3))
        self.swing_phase = np.zeros(6)  # 0-1 swing phase for each foot
        self.swing_height = 80.0  # mm
        self.swing_duration = 1.5  # seconds

        # Cache world positions for support feet (they should not move in world frame)
        self.support_foot_world_positions = np.zeros((6, 3))  # World frame positions for support feet

        # Initialize foot positions using the extracted constants
        self.foot_positions = HEXAPOD201_DEFAULT_FOOT_POSITIONS.copy()

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
            while (angle_diff[2] > math.pi):
                angle_diff[2] -= 2 * math.pi
            while (angle_diff[2] < -math.pi):
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
                self.current_pose.position.x = start_pose.position.x + \
                    (self.target_body_motion[0] / 1000.0) * progress  # Convert mm to m
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
                self._vis_feet()

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
                self._vis_feet()

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

    def _vis_feet(self):
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

    def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "10.1.180.190.1.1"):
        super().__init__(node_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.world_frame = "world"
        self.base_frame = "base_link"

        self.robot_pose_sub = rospy.Subscriber('/Odometry', Odometry, self._robot_pose_sub_callback)
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
            # Wait for enable
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
        # print(
        #     f"robot current pose updated from TF! "
        #     f"x: {self.current_pose.position.x}"
        #     f"y: {self.current_pose.position.y}"
        #     f"z: {self.current_pose.position.z}"
        # )
        return

    # Timer callbacks
    def update_feedback(self, event):
        """Periodic update of feedback from PLC"""
        super().update_feedback(event)

        if not self.plc_connected or not self._is_plc_enabled():
            return
        # Update current pose (use odom for now)
        # self._update_current_pose_from_plc()

        # Update foot positions
        self._update_footpos_from_plc()

    # @deprecated("PLC pose is not correct, update from odom instead.")
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

                # Convert Euler angles to quaternion
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
            self.symbol_Cmd_Time.write(self.cmdTime)  # type: ignore

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
            self.cmdPose["Pitch"] = euler[1]  # type: ignore
            self.cmdPose["Yaw"] = euler[2]   # type: ignore

            self.cmdPose["FG"] = 0  # type: ignore   # Movement mode
            self.cmdPose["Res"] = 0  # type: ignore
            self.symbol_Cmd_Pose.write(self.cmdPose)  # type: ignore

            # Start movement
            self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)  # type: ignore

            rospy.loginfo(f"Started movement to pose: {target_pose.position}")
            return True

        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
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
        self.symbol_Cmd_Time.write(self.cmdTime)  # type: ignore

        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # type: ignore    # Synchronous gait 1 是正常的2 3 6 步态
        self.cmdGait["GaitDF"] = 0.5  # type: ignore     # Duty factor 0.5就是2步态 0.667 就是三步态 0.833就是六步态
        self.cmdGait["SwapHigh"] = 100.0  # type: ignore     # Swing height (mm) 摆动高度
        self.cmdGait["LegNum"] = 0  # type: ignore    # Force control mode # 单腿运动的时候控制这个，这个先给0
        self.cmdGait["ForceMode"] = 0  # type: ignore    # 无力控是0
        self.cmdGait["Res"] = 0  # type: ignore        # 给0的话倍福的程序会再算一次足端的步长啥的，所以最好给1
        self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore

        # Set pose parameters
        self.cmdPose["X"] = max(-380, min(380, (aim_pose.pose.position.x - cur_pose.position.x) * 1000)
                                )  # type: ignore    # Convert to mm
        self.cmdPose["Y"] = max(-100, min(100, (aim_pose.pose.position.y - cur_pose.position.y) * 1000))  # type: ignore

        self.cmdPose["Z"] = 0.0  # type: ignore
        print(
            f"[move to] cmdPose set is: x: {(aim_pose.pose.position.x - cur_pose.position.x) * 1000}, y: {(aim_pose.pose.position.y - cur_pose.position.y) * 1000}")
        print(f"[move to] cmdPose set is clamped to: x: {self.cmdPose['X']}, y: {self.cmdPose['Y']}")

        if (use_virtual_odom):
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
        self.cmdPose["Pitch"] = 0.0  # type: ignore
        self.cmdPose["Yaw"] = 0.0   # type: ignore

        self.cmdPose["FG"] = 0  # type: ignore   # Movement mode 在发送的时候代表走一步停一下，1代表连续走
        self.cmdPose["Res"] = 0  # type: ignore    # Res = 0, 代表自动计算步长等参数
        self.symbol_Cmd_Pose.write(self.cmdPose)  # type: ignore    # write相当于发送

        # Start movement
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)  # type: ignore
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0:  # type: ignore
            # print(f"cur_beifu_Cmd['FG'] != 0, sendCtrlCmd.STOP_MOV, cur_beifu_Cmd['FG'] is {cur_beifu_Cmd['FG']}")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)  # type: ignore

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
        self.symbol_Cmd_Time.write(self.cmdTime)  # type: ignore

        # Set gait parameters
        self.cmdGait["GaitMode"] = 1  # type: ignore    # Synchronous gait 1 是正常的2 3 6 步态
        self.cmdGait["GaitDF"] = 0.5  # type: ignore     # Duty factor 0.5就是2步态 0.667 就是三步态 0.833就是六步态
        self.cmdGait["SwapHigh"] = 100.0  # type: ignore     # Swing height (mm) 摆动高度
        self.cmdGait["LegNum"] = 0  # type: ignore    # Force control mode # 单腿运动的时候控制这个，这个先给0
        self.cmdGait["ForceMode"] = 0  # type: ignore    # 无力控是0
        self.cmdGait["Res"] = 0  # type: ignore        # 给0的话倍福的程序会再算一次足端的步长啥的，所以最好给1
        self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore

        # Set pose parameters
        self.cmdPose["X"] = 0.0  # type: ignore    # Convert to mm
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
        self.cmdPose["Pitch"] = 0.0  # type: ignore
        yaw_diff: float = self.calPosYawDiff2d(aim_pose.pose, self.current_pose)
        yaw_control = -yaw_diff
        self.cmdPose["Yaw"] = max(-5.0/180.0*math.pi, min(5.0/180.0*math.pi, yaw_control))   # type: ignore

        print(f"[move to] cmdPose set is: yaw: {yaw_control}")
        print(f"[move to] cmdPose set is clamped to: yaw: {self.cmdPose['Yaw']}")         # type: ignore

        if (use_virtual_odom):
            # 虚拟地将移动后的期望位置更新为机器人的当前位置
            last_yaw: float = self.calPosYaw2d(self.current_pose)
            cur_yaw: float = last_yaw + self.cmdPose['Yaw']
            cur_yaw_quant = self._calQuanfromyaw(cur_yaw)
            self.current_pose.orientation = cur_yaw_quant

        self.cmdPose["FG"] = 0  # type: ignore   # Movement mode 在发送的时候代表走一步停一下，1代表连续走
        self.cmdPose["Res"] = 0  # type: ignore    # Res = 0, 代表自动计算步长等参数
        self.symbol_Cmd_Pose.write(self.cmdPose)  # type: ignore    # write相当于发送

        # Start movement
        self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)  # type: ignore
        time.sleep(0.005)
        cur_beifu_Cmd = self.symbol_PTCmdPos.read()
        while cur_beifu_Cmd["FG"] != 0:  # type: ignore
            # print(f"cur_beifu_Cmd['FG'] != 0, sendCtrlCmd.STOP_MOV, cur_beifu_Cmd['FG'] is {cur_beifu_Cmd['FG']}")
            cur_beifu_Cmd = self.symbol_PTCmdPos.read()
            time.sleep(0.005)
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)  # type: ignore

        # rospy.loginfo(f"Started movement to pose: {target_pose.position}")
        return True

    def follow_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:  # type: ignore
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value == 7:  # type: ignore
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
                # 拿一下机器人当前的位置
                cur_pose: Pose = self.get_current_pose()
                print(
                    f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose)) < self.admit_yaw_limit:
                    # 说明已经到达了当前点
                    print(
                        f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    # 如果有yaw角度diff 优先动yaw
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=False)
                    else:
                        print(
                            f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=False)

        return True

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        print("entered: [follow_virtual_trajectory]")
        if trajectory.poses is None:
            rospy.logerr("Trajectory poses are None")
            return False
        while self.symbol_QState.value != 2:  # type: ignore
            print(f"cur symbol_QState is: {self.symbol_QState}, try to enable PLC")
            if self.symbol_QState.value == 7:  # type: ignore
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
                # 拿一下机器人当前的位置
                cur_pose: Pose = self.get_current_pose()
                print(
                    f"[follow_traj]cur_pose x= {cur_pose.position.x} y= {cur_pose.position.y} z= {cur_pose.position.z} yaw= {self.calPosYaw2d(cur_pose)/math.pi*180.0}")
                # 模拟实际的SLAM反馈中到达目标点的判断函数, 认为走出一步之后就到达了目标点
                if self.calPosDisDiff2d(aim_pose, cur_pose) <= self.admit_pose_limit and abs(self.calPosYaw2d(cur_pose)) < self.admit_yaw_limit:
                    # 说明已经到达了当前点
                    print(
                        f"[follow_traj]has arrived aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                    cur_step += 1
                else:
                    # 如果有yaw角度diff 优先动yaw
                    if abs(self.calPosYaw2d(cur_pose)) > self.admit_yaw_limit:
                        print(f"[follow_traj]begin move to yaw: 0.0")
                        self.move_to_yaw(cur_pose, aim_pose, use_virtual_odom=True)
                    else:
                        print(
                            f"[follow_traj]begin move to aim_pose x: {aim_pose.pose.position.x}, y: {aim_pose.pose.position.y}, z: {aim_pose.pose.position.z}")
                        self.move_to_pos(cur_pose, aim_pose, use_virtual_odom=True)

        return True

    # Free gait methods
    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod using free gait with custom foothold definitions"""
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False

        # Enable PLC for free gait
        if not self._enable_plc():
            return False

        # Read current parameters
        if not self._read_plc_parameters():
            return False

        # Set free gait parameters
        self.cmdTime["TA"] = 1.5  # type: ignore # Acceleration time
        self.cmdTime["TM"] = 1.5  # type: ignore # Movement time
        self.cmdTime["TD"] = 0.0  # type: ignore # Deceleration overlap
        self.cmdTime["TZ"] = 0.6  # type: ignore # Z advance time
        self.symbol_Cmd_Time.write(self.cmdTime)

        # Set gait parameters for free gait
        self.cmdGait["GaitMode"] = 5  # Free gait mode
        self.cmdGait["GaitDF"] = 0.5  # Duty factor
        self.cmdGait["SwapHigh"] = 300.0  # Swing height (mm)
        self.cmdGait["LegNum"] = 0  # Leg number
        self.cmdGait["ForceMode"] = 0  # Force control mode
        self.cmdGait["Res"] = 0
        self.symbol_Cmd_Gait.write(self.cmdGait)

        # Start remote free gait movement
        self.symbol_CtrlCmd.write(CtrlCmd.REMOTE_MOV)

        # Wait for CPP to be ready for command
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

        # Prepare free gait command
        if self.ReqPTCmd is None:
            self.ReqPTCmd = self.symbol_ReqPTCmd.read()

        # Set body motion
        if self.keepConstBaseFootZ:
            body_motion[2] = 0.0  # Keep Z motion zero if flag is set
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
        # Start movement
        self.symbol_ReqFlag.write(2)  # Start movement

        # Update internal foot positions
        # self.foot_positions = foot_positions.copy()
        self._update_footpos_from_plc()
        self.target_foot_positions = foot_positions.copy()
        self.foot_support_flags = foot_flags.copy()

        rospy.loginfo(f"Started free gait movement with body motion: {body_motion}")
        return True

    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Move hexapod to target pose with specific foot positions using free gait"""
        if not self.plc_connected or not self.cpp_connected:
            rospy.logerr("PLC or CPP not connected")
            return False

        # Enable PLC for free gait
        if not self._enable_plc():
            return False

        # Read current parameters
        if not self._read_plc_parameters():
            return False

        # Calculate body motion from current to target pose
        # print("targetpos:", target_pose.position.x,target_pose.position.y,target_pose.position.z)
        # print("currpos:", self.current_pose.position.x,self.current_pose.position.y,self.current_pose.position.z)
        # print("foot positions:", foot_positions)
        # print("foot_flat:", foot_flags)
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
            # self.current_pose = copy.deepcopy(target_pose)
            rospy.loginfo("Coordinated pose and foot movement completed")

        return success

    def setCmd(self, **kwargs) -> bool:
        """Set detailed movement parameters"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return False

        try:
            # Time parameters
            if 'TA' in kwargs:
                self.cmdTime["TA"] = kwargs['TA']  # type: ignore
            if 'TM' in kwargs:
                self.cmdTime["TM"] = kwargs['TM']  # type: ignore
            if 'TD' in kwargs:
                self.cmdTime["TD"] = kwargs['TD']  # type: ignore
            if 'TZ' in kwargs:
                self.cmdTime["TZ"] = kwargs['TZ']  # type: ignore

            # Gait parameters
            if 'GaitMode' in kwargs:
                self.cmdGait["GaitMode"] = kwargs['GaitMode']  # type: ignore
            if 'GaitDF' in kwargs:
                self.cmdGait["GaitDF"] = kwargs['GaitDF']  # type: ignore
            if 'SwapHigh' in kwargs:
                self.cmdGait["SwapHigh"] = kwargs['SwapHigh']  # type: ignore
            if 'ForceMode' in kwargs:
                self.cmdGait["ForceMode"] = kwargs['ForceMode']  # type: ignore

            # Write parameters to PLC
            if self.cmdTime:
                self.symbol_Cmd_Time.write(self.cmdTime)  # type: ignore
            if self.cmdGait:
                self.symbol_Cmd_Gait.write(self.cmdGait)  # type: ignore

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
            self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)  # type: ignore
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

        # rospy.loginfo("Joint encoder timer callback triggered")
        pass

    # Setter/Getter
    def robot_is_moving(self):
        """Check if the robot is currently moving based on PLC state"""
        if self.symbol_ReqFlag is not None:
            return not self.symbol_ReqFlag.value
        else:
            return 0

    # Debug
    def test_footpos_read(self):
        """Test reading foot positions from PLC"""
        if not self.plc_connected:
            rospy.logerr("PLC not connected")
            return

        try:
            act_pos = self.symbol_PTActPos.read()
            if act_pos:
                rospy.loginfo(
                    f"Current Pose from PLC: X={act_pos['X']}, Y={act_pos['Y']}, Z={act_pos['Z']}, Roll={act_pos['Roll']}, Pitch={act_pos['Pitch']}, Yaw={act_pos['Yaw']}")
            else:
                rospy.logwarn("No pose data received from PLC")
        except Exception as e:
            rospy.logerr(f"Failed to read pose from PLC: {str(e)}")


def main():
    """Main function to run hexapod interface"""
    # Initialize ROS node first
    rospy.init_node('hexapod201_interface', anonymous=True)

    # Get parameters from ROS parameter server
    interface_type = rospy.get_param('~interface_type', None)
    use_dummy = rospy.get_param('~dummy', False)
    plc_ip = rospy.get_param('~plc_ip', '10.1.180.190.1.1')
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
    interface = Hexapod201Interface(node_name="hexapod201_interface", plc_ip="10.1.180.190.1.1")
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


def test_pose_with_feet(gait2phase=0):
    """Test move_to_pose_with_feet method"""
    rospy.init_node('test_hexapod_pose_with_feet', anonymous=True)
    interface = Hexapod201Interface(node_name="hexapod201_interface", plc_ip="192.168.1.115.1.1")

    # Define target pose - move forward 0.2m and turn 30 degrees
    target_pose = Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.05  # Lift body slightly

    # Convert 30 degrees to radians and create quaternion
    yaw_angle = np.pi / 6  # 30 degrees in radians
    quat = quaternion_from_euler(0.0, 0.0, yaw_angle)
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    # Define custom foot positions (in mm, body frame)
    # Move some feet to new positions for stepping pattern
    foot_positions = np.array([
        [760, 1096.2, -405],   # Foot 1 - step forward
        [0, 1251.2, -405],     # Foot 2 - keep in place
        [-560, 1096.2, -405],  # Foot 3 - step back slightly
        [760, -896.2, -405],  # Foot 4 - step forward
        [0, -1251.2, -405],   # Foot 5 - keep in place
        [-560, -896.2, -405]  # Foot 6 - step back slightly
    ])

    # Define foot support flags (0=support, 1=swing)
    # Alternate pattern: feet 1, 3, 5 swing, feet 2, 4, 6 support
    if gait2phase == 0:
        foot_flags = np.array([1, 0, 1, 0, 1, 0])
    else:
        foot_flags = np.array([0, 1, 0, 1, 0, 1])

    rospy.loginfo("Starting coordinated pose and foot movement test...")
    rospy.loginfo(f"Target pose: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
    rospy.loginfo(f"Target yaw: {yaw_angle} rad ({np.degrees(yaw_angle)} deg)")
    rospy.loginfo(f"Swing feet: {np.where(foot_flags == 1)[0] + 1}")  # +1 for 1-indexed foot numbering

    # Execute coordinated movement
    success = interface.move_to_pose_with_feet(target_pose, foot_positions, foot_flags)

    if success:
        rospy.loginfo("Movement command sent successfully")
        rospy.sleep(8)  # Wait for movement to complete (longer time for coordinated movement)

        # Get final pose
        current_pose = interface.get_current_pose()
        rospy.loginfo(
            f"Final pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")

        # Get final foot positions
        interface.update_footpos()  # Update foot positions from PLC
        final_foot_positions = interface.get_foot_positions()
        interface.test_footpos_read()  # Read and log current pose from PLC
        rospy.loginfo("Final foot positions:")
        for i, pos in enumerate(final_foot_positions):
            rospy.loginfo(f"  Foot {i+1}: x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.1f} mm")
    else:
        rospy.logerr("Failed to execute coordinated movement")


if __name__ == "__main__":
    # main()
    test_interface()
    # test_pose_with_feet(0)
    # test_pose_with_feet(1)
