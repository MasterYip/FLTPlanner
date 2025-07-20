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

# Import the ROS visualizer
from ros_visualizer import ROSVisualizer


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
        
        # Initialize ROS node
        # rospy.init_node(node_name, anonymous=True)
        
        # Publishers and subscribers
        self.pose_pub = rospy.Publisher('/hexapod/current_pose', PoseStamped, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pose_cmd_sub = rospy.Subscriber('/hexapod/pose_cmd', PoseStamped, self.pose_cmd_callback)
        
        # Timer for publishing current pose
        self.pose_timer = rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        
        # Visualization
        self.visualizer = ROSVisualizer("odom", "hexapod_visualization")
        
        rospy.loginfo(f"{node_name} initialized")
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands by integrating to get target pose"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_cmd_time).to_sec()
        self.last_cmd_time = current_time
        
        if dt > 0:
            # Integrate velocity to get position change
            linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
            
            # Simple Euler integration
            self.cmd_vel_integration[:3] += linear_vel * dt
            self.cmd_vel_integration[3:] += angular_vel * dt
            
            # Create target pose from current pose + integration
            self.target_pose.position.x = self.current_pose.position.x + self.cmd_vel_integration[0]
            self.target_pose.position.y = self.current_pose.position.y + self.cmd_vel_integration[1]
            self.target_pose.position.z = self.current_pose.position.z + self.cmd_vel_integration[2]
            
            # Convert Euler angles to quaternion
            roll, pitch, yaw = self.cmd_vel_integration[3], self.cmd_vel_integration[4], self.cmd_vel_integration[5]
            quat = quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.orientation.w = quat[0]
            self.target_pose.orientation.x = quat[1]
            self.target_pose.orientation.y = quat[2]
            self.target_pose.orientation.z = quat[3]
            
            # Execute movement
            self.move_to_pose(self.target_pose)
    
    def pose_cmd_callback(self, msg: PoseStamped):
        """Handle direct pose commands"""
        self.target_pose = msg.pose
        self.move_to_pose(self.target_pose)
    
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
        self.visualizer.del_cube()
        
        # Create box at current pose
        position = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        # Convert quaternion to rotation matrix (simplified)
        quat = np.array([
            self.current_pose.orientation.w,
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z
        ])
        
        # Box size (hexapod body dimensions)
        box_size = 0.3  # 30cm cube
        self.visualizer.vis_cube(position, quat)
    
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
        self.movement_speed = 0.1  # m/s
        self.rotation_speed = 0.5  # rad/s
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
            start_pose = self.current_pose
            target_pose = self.target_pose
            
            # Calculate distance and angle differences
            pos_diff = np.array([
                target_pose.position.x - start_pose.position.x,
                target_pose.position.y - start_pose.position.y,
                target_pose.position.z - start_pose.position.z
            ])
            
            # Get current and target Euler angles
            start_euler = euler_from_quaternion([
                start_pose.orientation.w,
                start_pose.orientation.x,
                start_pose.orientation.y,
                start_pose.orientation.z
            ])
            
            target_euler = euler_from_quaternion([
                target_pose.orientation.w,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z
            ])
            
            angle_diff = np.array([
                target_euler[0] - start_euler[0],
                target_euler[1] - start_euler[1],
                target_euler[2] - start_euler[2]
            ])
            
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
                    self.current_pose.orientation.w = quat[0]
                    self.current_pose.orientation.x = quat[1]
                    self.current_pose.orientation.y = quat[2]
                    self.current_pose.orientation.z = quat[3]
                    
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
        
        # Connect to PLC
        self._connect_plc()
        
        rospy.loginfo("Real hexapod interface initialized")
    
    def _connect_plc(self):
        """Establish PLC connection"""
        try:
            self.plc = pyads.Connection(self.plc_ip, pyads.PORT_TC3PLC1)
            self.plc.open()
            
            # Initialize PLC symbols
            self.symbol_Cmd_Time = self.plc.get_symbol('MAIN.PTCmd.TM', structure_def=stTime_def)
            self.symbol_Cmd_Gait = self.plc.get_symbol('MAIN.PTCmd.Gait', structure_def=stGait_def)
            self.symbol_Cmd_Pose = self.plc.get_symbol('MAIN.PTCmd.Pose', structure_def=stPose_def)
            self.symbol_CtrlCmd = self.plc.get_symbol('MAIN.CtrlCmd', plc_datatype="UDINT")
            self.symbol_State = self.plc.get_symbol('MAIN.state', plc_datatype="UDINT")
            self.symbol_QState = self.plc.get_symbol('MAIN.Q_State')
            self.symbol_PTActPos = self.plc.get_symbol('MAIN.PTActPos', structure_def=stPose_def)
            
            # Enable auto-update for feedback
            self.symbol_QState.auto_update = True
            self.symbol_PTActPos.auto_update = True
            
            self.plc_connected = True
            rospy.loginfo("PLC connection established")
            
        except Exception as e:
            rospy.logerr(f"PLC connection failed: {str(e)}")
            self.plc_connected = False
    
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
        
        try:
            # Enable PLC
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Set movement parameters
            self.cmdTime["TA"] = 0.5  # Acceleration time
            self.cmdTime["TM"] = 1.5  # Movement time
            self.cmdTime["TD"] = 0.0  # Deceleration overlap
            self.cmdTime["TZ"] = 0.0  # Z advance time
            self.symbol_Cmd_Time.write(self.cmdTime)
            
            # Set gait parameters
            self.cmdGait["GaitMode"] = 1  # Synchronous gait
            self.cmdGait["GaitDF"] = 0.5  # Duty factor
            self.cmdGait["SwapHigh"] = 80.0  # Swing height (mm)
            self.cmdGait["LegNum"] = 0  # Force control mode
            self.cmdGait["ForceMode"] = 0
            self.cmdGait["Res"] = 0
            self.symbol_Cmd_Gait.write(self.cmdGait)
            
            # Set pose parameters
            self.cmdPose["X"] = target_pose.position.x * 1000  # Convert to mm
            self.cmdPose["Y"] = target_pose.position.y * 1000
            self.cmdPose["Z"] = target_pose.position.z * 1000
            
            # Convert quaternion to Euler angles
            euler = euler_from_quaternion([
                target_pose.orientation.w,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z
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
            
        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
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
        """Cleanup PLC connection"""
        if self.plc_connected and self.plc:
            try:
                self.symbol_State.write(State.DISENABLE)
                self.plc.close()
                rospy.loginfo("PLC connection closed")
            except Exception as e:
                rospy.logerr(f"Error closing PLC connection: {str(e)}")


def main():
    """Main function to run hexapod interface"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Hexapod201 Interface')
    parser.add_argument('--dummy', action='store_true', help='Use dummy interface for simulation')
    parser.add_argument('--plc_ip', type=str, default='5.157.100.214.1.1', help='PLC IP address')
    parser.add_argument('--node_name', type=str, default='hexapod201_interface', help='ROS node name')
    
    args = parser.parse_args()
    
    try:
        if args.dummy:
            interface = DummyHexapod201Interface(args.node_name)
        else:
            interface = Hexapod201Interface(args.node_name, args.plc_ip)
        
        rospy.loginfo("Hexapod interface started")
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down hexapod interface")
        if hasattr(interface, 'cleanup'):
            interface.cleanup()
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")


if __name__ == "__main__":
    main()
