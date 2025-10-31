import copy
import threading
from hexapod201_base_interface import *

class DummyHexapod201Interface(Hexapod201BaseInterface):
    """Dummy interface for simulation/testing"""

    def cleanup(self):
        """Dummy cleanup method"""
        rospy.loginfo("Dummy interface cleanup called")
    
    def __init__(self, node_name: str = "dummy_hexapod201_interface"):
        super().__init__(node_name)
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

    def follow_trajectory(self, trajectory: Path) -> bool:
        """Simulate following a trajectory"""
        rospy.loginfo("Dummy interface following trajectory")
        return True

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        """Simulate virtual following of a trajectory"""
        rospy.loginfo("Dummy interface virtually following trajectory")
        return True

    
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
            if angle_diff[2] > math.pi:
                angle_diff[2] -= 2 * math.pi
            elif angle_diff[2] < -math.pi:
                angle_diff[2] += 2 * math.pi
            
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
