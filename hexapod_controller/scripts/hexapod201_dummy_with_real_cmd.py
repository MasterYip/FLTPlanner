#!/usr/bin/env python3

import rospy
import numpy as np
from hexapod201_dummy_interface import DummyHexapod201Interface
from hexapod201_interface import Hexapod201Interface
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from typing import Optional


class DummyWithRealCmdHexapod201Interface(DummyHexapod201Interface):
    """
    Hybrid interface that uses dummy feedback but sends commands to real robot.
    Useful for testing command sequences while getting predictable feedback.
    """
    
    def __init__(self, node_name: str = "dummy_with_real_cmd_hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
        # Initialize dummy interface for all feedback and simulation
        super().__init__(node_name)
        
        # Initialize real interface for command sending only
        self.real_interface: Optional[Hexapod201Interface] = None
        self.plc_ip = plc_ip
        self.real_cmd_enabled = True
        
        # Try to connect to real robot
        self._init_real_interface()
        
        rospy.loginfo("Dummy with real command interface initialized")

    def _init_real_interface(self):
        """Initialize real robot interface for command sending"""
        try:
            self.real_interface = Hexapod201Interface(
                node_name=f"{self.node_name}_real_cmd", 
                plc_ip=self.plc_ip
            )
            rospy.loginfo("Real robot interface connected for command sending")
            self.real_cmd_enabled = True
        except Exception as e:
            rospy.logwarn(f"Failed to connect to real robot: {str(e)}")
            rospy.logwarn("Commands will only be simulated")
            self.real_cmd_enabled = False
            self.real_interface = None

    def cleanup(self):
        """Cleanup both dummy and real interfaces"""
        super().cleanup()
        if self.real_interface:
            self.real_interface.cleanup()

    # Override movement methods to send commands to real robot while using dummy feedback
    
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Send command to real robot and simulate dummy movement"""
        success = True
        
        # Send command to real robot if available
        if self.real_cmd_enabled and self.real_interface:
            try:
                real_success = self.real_interface.move_to_pose(target_pose)
                if not real_success:
                    rospy.logwarn("Real robot command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending command to real robot: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        dummy_success = super().move_to_pose(target_pose)
        
        return dummy_success and success

    def follow_trajectory(self, trajectory: Path) -> bool:
        """Send trajectory to real robot and simulate dummy movement"""
        success = True
        
        # Send command to real robot if available
        if self.real_cmd_enabled and self.real_interface:
            try:
                # Don't wait for real robot to complete, just send the command
                real_success = self.real_interface.follow_trajectory(trajectory)
                if not real_success:
                    rospy.logwarn("Real robot trajectory command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending trajectory to real robot: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        dummy_success = super().follow_trajectory(trajectory)
        
        return dummy_success and success

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        """Send virtual trajectory to real robot and simulate dummy movement"""
        success = True
        
        # Send command to real robot if available
        if self.real_cmd_enabled and self.real_interface:
            try:
                # Use the real robot's follow_trajectory method (not virtual)
                real_success = self.real_interface.follow_trajectory(trajectory)
                if not real_success:
                    rospy.logwarn("Real robot virtual trajectory command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending virtual trajectory to real robot: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        dummy_success = super().follow_virtual_trajectory(trajectory)
        
        return dummy_success and success

    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send free gait command to real robot and simulate dummy movement"""
        success = True
        
        # Send command to real robot if available
        if self.real_cmd_enabled and self.real_interface:
            try:
                real_success = self.real_interface.move_free_gait(body_motion, foot_positions, foot_flags)
                if not real_success:
                    rospy.logwarn("Real robot free gait command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending free gait command to real robot: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        dummy_success = super().move_free_gait(body_motion, foot_positions, foot_flags)
        
        return dummy_success and success

    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send coordinated pose and foot command to real robot and simulate dummy movement"""
        success = True
        
        # Send command to real robot if available
        if self.real_cmd_enabled and self.real_interface:
            try:
                real_success = self.real_interface.move_to_pose_with_feet(target_pose, foot_positions, foot_flags)
                if not real_success:
                    rospy.logwarn("Real robot coordinated movement command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending coordinated movement to real robot: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        dummy_success = super().move_to_pose_with_feet(target_pose, foot_positions, foot_flags)
        
        return dummy_success and success

    def setCmd(self, **kwargs) -> bool:
        """Set parameters on both real and dummy interfaces"""
        # Set parameters on dummy interface
        dummy_success = super().setCmd(**kwargs)
        
        # Set parameters on real interface if available
        real_success = True
        if self.real_cmd_enabled and self.real_interface:
            try:
                real_success = self.real_interface.setCmd(**kwargs)
            except Exception as e:
                rospy.logwarn(f"Error setting parameters on real robot: {str(e)}")
                real_success = False
        
        return dummy_success and real_success

    def stop_movement(self) -> bool:
        """Stop movement on both real and dummy interfaces"""
        # Stop dummy movement
        dummy_success = super().stop_movement()
        
        # Stop real robot if available
        real_success = True
        if self.real_cmd_enabled and self.real_interface:
            try:
                real_success = self.real_interface.stop_movement()
            except Exception as e:
                rospy.logwarn(f"Error stopping real robot: {str(e)}")
                real_success = False
        
        return dummy_success and real_success

    # Utility methods for debugging and control
    
    def enable_real_commands(self) -> bool:
        """Enable sending commands to real robot"""
        if not self.real_interface:
            self._init_real_interface()
        
        self.real_cmd_enabled = True
        rospy.loginfo("Real robot commands enabled")
        return self.real_interface is not None

    def disable_real_commands(self):
        """Disable sending commands to real robot (dummy mode only)"""
        self.real_cmd_enabled = False
        rospy.loginfo("Real robot commands disabled - dummy mode only")

    def get_real_robot_status(self) -> dict:
        """Get status information about real robot connection"""
        return {
            'real_interface_available': self.real_interface is not None,
            'real_cmd_enabled': self.real_cmd_enabled,
            'plc_ip': self.plc_ip,
            'plc_connected': self.real_interface.plc_connected if self.real_interface else False,
            'cpp_connected': self.real_interface.cpp_connected if self.real_interface else False,
        }

    def robot_is_moving(self) -> bool:
        """
        Override to always use dummy interface feedback for consistency.
        Real robot movement status is not used for feedback.
        """
        return super().robot_is_moving()