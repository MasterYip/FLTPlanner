from abc import ABC, abstractmethod
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped, Pose
import geometry_msgs.msg  # Add this import for Point and Vector3
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path

# Import the ROS visualizer
from ros_visualizer import ROSVisualizer, VisStyle  # pyright: ignore[reportAttributeAccessIssue]
# Import FootState message
from legged_traj_plan.msg import FootState
from std_msgs.msg import Bool

# Hexapod201 Default Configuration Constants
HEXAPOD201_DEFAULT_FOOT_POSITIONS = np.array([
    [660, -996.2, -405],  # Foot201 4 - FootElAir 0
    [0, -1251.2, -405],   # Foot201 5 - FootElAir 1
    [-660, -996.2, -405], # Foot201 6 - FootElAir 2
    [660, 996.2, -405],   # Foot201 1 - FootElAir 3
    [0, 1251.2, -405],    # Foot201 2 - FootElAir 4
    [-660, 996.2, -405],  # Foot201 3 - FootElAir 5
])

HEXAPOD201_DEFAULT_BODY_HEIGHT = 0.405  # meters
HEXAPOD201_DEFAULT_INIT_POSE = {
    'position': {'x': 0.0, 'y': 0.0, 'z': HEXAPOD201_DEFAULT_BODY_HEIGHT},
    'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
}

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
        self.dt = 1.0
        
        # ROS node should be initialized before creating this class
        
        # Publishers and subscribers
        self.pose_pub = rospy.Publisher('/hexapod/current_pose', PoseStamped, queue_size=10)
        self.foot_state_pub = rospy.Publisher('/hexapod/foot_state', FootState, queue_size=10)
        self.robot_is_moving_pub = rospy.Publisher('/robot_is_moving', Bool, queue_size=10)
        # self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/pose_cmd', PoseStamped, self.pose_cmd_callback)
        self.foot_cmd_sub = rospy.Subscriber('/hexapod/foot_cmd', FootState, self.foot_cmd_callback)
        self.path_cmd_sub = rospy.Subscriber('/hexapod/path_cmd', Path, self.follow_trajectory)
        
        # Timer for publishing current pose and foot state
        self.pub_timer = rospy.Timer(rospy.Duration(0, int(1e8)), self.publish_feedback)
        self.update_timer = rospy.Timer(rospy.Duration(0, int(5e7)), self.update_feedback)
        

        # Visualization
        self.visualizer = ROSVisualizer("world", "hexapod_visualization")
        
        # Free gait parameters
        self.foot_positions = np.zeros((6, 3))  # Current foot positions [x, y, z] in mm
        self.target_foot_positions = np.zeros((6, 3))  # Target foot positions [x, y, z] in mm
        self.foot_support_flags = np.zeros(6, dtype=int)  # 0=support, 1=swing
        self.default_foot_positions = HEXAPOD201_DEFAULT_FOOT_POSITIONS.copy()
        # self.foot_positions = self.default_foot_positions.copy()
        self.target_foot_positions = self.default_foot_positions.copy()

    def cleanup(self):
        """cleanup method"""
        pass

    # Callbacks
    def cmd_vel_callback(self, msg: Twist):
        """
        Handle velocity commands by integrating to get target pose
        接收线速度和角速度指令, 根据时间步长dt积分为期望的位置增量，加上当前位置求出期望位置, 之后
        调用self.move_to_pose()函数执行运动
        """
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
    
    def pose_cmd_callback(self, msg: PoseStamped):
        """
        Cache pose command - movement will be triggered by foot state callback
        直接给出目标位置, 然后调用self.move_to_pose()函数执行运动
        """
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
    
    # Timer callbacks
    def update_feedback(self, event):
        """Update internal state for feedback publishing"""
        # This function can be expanded to update the current pose and foot states
        # For now, it just ensures that the publish_feedback function is called periodically

        # Visualization
        self.vis_update()

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

        # Robot is moving status
        self.robot_is_moving_pub.publish(Bool(self.robot_is_moving()))

    # Motion Interface
    @abstractmethod
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Move hexapod to target pose - to be implemented by subclasses"""
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

    # Setter/Getter
    def robot_is_moving(self):
        return self.is_moving

    def get_foot_positions(self) -> np.ndarray:
        """Get current foot positions"""
        return self.foot_positions.copy()
    
    def get_target_foot_positions(self) -> np.ndarray:
        """Get target foot positions"""
        return self.target_foot_positions.copy()
  
    def get_current_pose(self) -> Pose:
        """Get current pose"""
        return self.current_pose
    
    def get_target_pose(self) -> Pose:
        """Get target pose"""
        return self.target_pose

    def set_default_foot_positions(self, positions: np.ndarray):
        """Set default foot positions for support stance"""
        if positions.shape == (6, 3):
            self.default_foot_positions = positions.copy()

    def reset_integration(self):
        """Reset velocity integration"""
        self.cmd_vel_integration = np.zeros(6)
  
    # Utils
    def calPosDisDiff3d(self, a:Pose, b:Pose) -> float:
        """Calculate 3D distance between two poses, handling Pose and PoseStamped."""
        if isinstance(a, PoseStamped):
            a = a.pose
        if isinstance(b, PoseStamped):
            b = b.pose
        return pow(pow(a.position.x - b.position.x, 2) +
                   pow(a.position.y - b.position.y, 2) +
                   pow(a.position.z - b.position.z, 2), 0.5)
    
    def calPosDisDiff2d(self, a:Pose, b:Pose) -> float:
        """Calculate 2D distance between two poses, handling Pose and PoseStamped."""
        if isinstance(a, PoseStamped):
            a = a.pose
        if isinstance(b, PoseStamped):
            b = b.pose
        return pow(pow(a.position.x - b.position.x, 2) +
                    pow(a.position.y - b.position.y, 2), 0.5)
        
    def calPosYawDiff2d(self, a: Pose, b: Pose) -> float:
        """return yaw_b - yaw_a"""
        def extract_quaternion(pose):
            if isinstance(pose, Pose):
                return [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            elif isinstance(pose, PoseStamped):
                return [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
            else:
                raise TypeError("Input must be of type Pose or PoseStamped")
        
        quat_a = extract_quaternion(a)
        quat_b = extract_quaternion(b)
    
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
        if isinstance(a, Pose):
            quat_a = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
        elif isinstance(a, PoseStamped):
            quat_a = [a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w]
        else:
            raise TypeError("Input must be of type Pose or PoseStamped")
        # Convert quaternions to Euler angles
        _, _, yaw_a = euler_from_quaternion(quat_a)
        
        while yaw_a > math.pi:
            yaw_a -= 2 * math.pi
        while yaw_a < -math.pi:
            yaw_a += 2 * math.pi
        return yaw_a
    
    def _calQuanfromyaw(self, yaw: float) -> geometry_msgs.msg.Quaternion:
        """Convert yaw angle to a quaternion."""
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        return geometry_msgs.msg.Quaternion(*quat)
    
    # Visualization
    def vis_update(self):
        # Clear previous visualization
        self.visualizer.del_all()
        self._vis_body()
        self._vis_feet()

    def _vis_body(self):
        """Visualize hexapod body as a box in RViz"""
        
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

    # Debug
    def publish_current_pose(self, event):
        """
        Publish current pose for visualization
        发布当前位置, 将当前位置在odom坐标系下发布, 并可视化一个长方体形状的六足机体
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)
        
        # Visualize hexapod body as a box
        self._vis_body()