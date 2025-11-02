#!/usr/bin/env python3

import rospy
import numpy as np
from hexapod201_dummy_interface import DummyHexapod201Interface
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from typing import Optional
import time
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import PLC communication modules
try:
    import pyads
    from beifu_control_vals import *
    PLC_AVAILABLE = True
except ImportError:
    rospy.logwarn("PLC modules not available, running in dummy-only mode")
    PLC_AVAILABLE = False


class DummyWithRealCmdHexapod201Interface(DummyHexapod201Interface):
    """
    Hybrid interface that uses dummy feedback but sends commands to real robot via PLC.
    Useful for testing command sequences while getting predictable feedback.
    """
    
    def __init__(self, node_name: str = "dummy_with_real_cmd_hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
        # Initialize dummy interface for all feedback and simulation
        super().__init__(node_name)
        
        # PLC connection parameters
        self.plc_ip = plc_ip
        self.plc = None
        self.cpp = None
        self.plc_connected = False
        self.cpp_connected = False
        self.real_cmd_enabled = True
        
        # PLC symbols (same as real interface)
        self.symbol_Cmd_Time = None
        self.symbol_Cmd_Gait = None
        self.symbol_Cmd_Pose = None
        self.symbol_CtrlCmd = None
        self.symbol_State = None
        self.symbol_QState = None
        self.symbol_PTActPos = None
        self.symbol_PTCmdPos = None
        
        # CPP symbols for free gait
        self.symbol_ReqPTCmd = None
        self.symbol_ReqFlag = None
        self.ReqPTCmd = None
        
        # Movement parameters
        self.cmdTime = None
        self.cmdGait = None
        self.cmdPose = None
        
        # Try to connect to real robot PLC
        if PLC_AVAILABLE:
            self._init_plc_connection()
        else:
            rospy.logwarn("PLC modules not available - commands will only be simulated")
            self.real_cmd_enabled = False
        
        rospy.loginfo("Dummy with real command interface initialized")

    def _init_plc_connection(self):
        """Initialize PLC connection for command sending"""
        try:
            # PLC connection
            self.plc = pyads.Connection(self.plc_ip, pyads.PORT_TC3PLC1, '192.168.3.101')
            self.plc.open()

            # CPP connection for free gait
            self.cpp = pyads.Connection(self.plc_ip, 351, '192.168.3.101')
            self.cpp.open()

            # Initialize PLC symbols (same as real interface)
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
            self.symbol_ReqPTCmd = self.cpp.get_symbol('CPP.Inputs.ReqPTCmd', structure_def=stPose_def)
            self.symbol_ReqFlag = self.cpp.get_symbol('CPP.Inputs.ReqFlag')
            
            # Enable auto-update for state checking
            self.symbol_QState.auto_update = True
            self.symbol_ReqFlag.auto_update = True
            self.symbol_PTCmdPos.auto_update = True
            
            self.plc_connected = True
            self.cpp_connected = True
            self.real_cmd_enabled = True
            rospy.loginfo("PLC connection established for command sending")
            
        except Exception as e:
            rospy.logwarn(f"Failed to connect to PLC: {str(e)}")
            rospy.logwarn("Commands will only be simulated")
            self.real_cmd_enabled = False
            self.plc_connected = False
            self.cpp_connected = False

    def cleanup(self):
        """Cleanup both dummy and PLC connections"""
        super().cleanup()
        if self.plc_connected and self.plc:
            try:
                if self.symbol_State:
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

    # PLC Helper Methods
    def _is_plc_enabled(self):
        """Check if PLC is enabled"""
        if not self.plc_connected or not self.symbol_QState:
            return False
        return self.symbol_QState.value == State.FEEDMOV

    def _enable_plc(self):
        """Enable PLC for movement"""
        if not self.plc_connected or not self.symbol_State:
            return False
        if self._is_plc_enabled():
            return True
        try:
            self.symbol_State.write(State.ENABLE)
            # Wait for enable with timeout
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
            rospy.logwarn(f"Failed to read PLC parameters: {str(e)}")
            return False

    # Override movement methods to send commands to real robot while using dummy feedback
    
    def move_to_pose(self, target_pose: Pose) -> bool:
        """Send command to real robot PLC and simulate dummy movement"""
        # Send command to real robot PLC if available
        if self.real_cmd_enabled and self.plc_connected:
            try:
                success = self._send_pose_to_plc(target_pose)
                if not success:
                    rospy.logwarn("Real robot PLC command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending command to real robot PLC: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        return super().move_to_pose(target_pose)

    def _send_pose_to_plc(self, target_pose: Pose) -> bool:
        """Send pose command to PLC"""
        try:
            # Enable PLC
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Set movement parameters
            self.cmdTime["TA"] = 1.5
            self.cmdTime["TM"] = 1.5
            self.cmdTime["TD"] = 0.0
            self.cmdTime["TZ"] = 0.2
            self.symbol_Cmd_Time.write(self.cmdTime)
            
            # Set gait parameters
            self.cmdGait["GaitMode"] = 1
            self.cmdGait["GaitDF"] = 0.5
            self.cmdGait["SwapHigh"] = 100.0
            self.cmdGait["LegNum"] = 0
            self.cmdGait["ForceMode"] = 0
            self.cmdGait["Res"] = 0
            self.symbol_Cmd_Gait.write(self.cmdGait)
            
            # Set pose parameters
            self.cmdPose["X"] = min(400, target_pose.position.x * 1000)  # Convert to mm
            self.cmdPose["Y"] = min(200, target_pose.position.y * 1000)
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
            
            self.cmdPose["FG"] = 0
            self.cmdPose["Res"] = 0
            self.symbol_Cmd_Pose.write(self.cmdPose)
            
            # Start movement
            self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)
            
            rospy.loginfo(f"Sent pose command to PLC: {target_pose.position}")
            return True
            
        except Exception as e:
            rospy.logerr(f"PLC pose command failed: {str(e)}")
            return False

    def follow_trajectory(self, trajectory: Path) -> bool:
        """Send trajectory to real robot PLC and simulate dummy movement"""
        # Send command to real robot PLC if available
        if self.real_cmd_enabled and self.plc_connected:
            try:
                success = self._send_trajectory_to_plc(trajectory)
                if not success:
                    rospy.logwarn("Real robot PLC trajectory command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending trajectory to real robot PLC: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        return super().follow_trajectory(trajectory)

    def _send_trajectory_to_plc(self, trajectory: Path) -> bool:
        """Send trajectory to PLC (non-blocking)"""
        try:
            if not trajectory.poses:
                return False
            
            # Enable PLC
            if not self._enable_plc():
                return False
            
            # Send first pose to start trajectory following
            first_pose = trajectory.poses[0].pose
            success = self._send_pose_to_plc(first_pose)
            
            rospy.loginfo(f"Started trajectory following on PLC with {len(trajectory.poses)} waypoints")
            return success
            
        except Exception as e:
            rospy.logerr(f"PLC trajectory command failed: {str(e)}")
            return False

    def follow_virtual_trajectory(self, trajectory: Path) -> bool:
        """Send virtual trajectory to real robot PLC and simulate dummy movement"""
        # Send command to real robot PLC if available  
        if self.real_cmd_enabled and self.plc_connected:
            try:
                success = self._send_trajectory_to_plc(trajectory)
                if not success:
                    rospy.logwarn("Real robot PLC virtual trajectory command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending virtual trajectory to real robot PLC: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        return super().follow_virtual_trajectory(trajectory)

    def move_free_gait(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send free gait command to real robot PLC and simulate dummy movement"""
        # Send command to real robot PLC if available
        if self.real_cmd_enabled and self.plc_connected and self.cpp_connected:
            try:
                success = self._send_free_gait_to_plc(body_motion, foot_positions, foot_flags)
                if not success:
                    rospy.logwarn("Real robot PLC free gait command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending free gait command to real robot PLC: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        return super().move_free_gait(body_motion, foot_positions, foot_flags)

    def _send_free_gait_to_plc(self, body_motion: np.ndarray, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send free gait command to PLC"""
        try:
            # Enable PLC for free gait
            if not self._enable_plc():
                return False
            
            # Read current parameters
            if not self._read_plc_parameters():
                return False
            
            # Set free gait parameters
            self.cmdTime["TA"] = 1.0
            self.cmdTime["TM"] = 1.5
            self.cmdTime["TD"] = 0.0
            self.cmdTime["TZ"] = 0.5
            self.symbol_Cmd_Time.write(self.cmdTime)
            
            # Set gait parameters for free gait
            self.cmdGait["GaitMode"] = 5  # Free gait mode
            self.cmdGait["GaitDF"] = 0.5
            self.cmdGait["SwapHigh"] = 100.0
            self.cmdGait["LegNum"] = 0
            self.cmdGait["ForceMode"] = 0
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
            self.ReqPTCmd["X"] = body_motion[0]  # mm
            self.ReqPTCmd["Y"] = body_motion[1]  # mm
            self.ReqPTCmd["Z"] = body_motion[2]  # mm
            self.ReqPTCmd["Roll"] = body_motion[3]  # rad
            self.ReqPTCmd["Pitch"] = body_motion[4]  # rad
            self.ReqPTCmd["Yaw"] = body_motion[5]  # rad
            self.ReqPTCmd["FG"] = 0
            self.ReqPTCmd["Res"] = 0
            
            # Set foot positions and flags (with proper remapping)
            FOOT_REMAP = [3, 4, 5, 0, 1, 2]  # Remap from FootElAir to Foot201
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
            
            rospy.loginfo(f"Sent free gait command to PLC: body_motion={body_motion}")
            return True
            
        except Exception as e:
            rospy.logerr(f"PLC free gait command failed: {str(e)}")
            return False

    def move_to_pose_with_feet(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send coordinated pose and foot command to real robot PLC and simulate dummy movement"""
        # Send command to real robot PLC if available
        if self.real_cmd_enabled and self.plc_connected and self.cpp_connected:
            try:
                success = self._send_pose_with_feet_to_plc(target_pose, foot_positions, foot_flags)
                if not success:
                    rospy.logwarn("Real robot PLC coordinated movement command failed, continuing with dummy simulation")
            except Exception as e:
                rospy.logwarn(f"Error sending coordinated movement to real robot PLC: {str(e)}")
        
        # Always execute dummy movement for predictable feedback
        return super().move_to_pose_with_feet(target_pose, foot_positions, foot_flags)

    def _send_pose_with_feet_to_plc(self, target_pose: Pose, foot_positions: np.ndarray, foot_flags: np.ndarray) -> bool:
        """Send coordinated pose and foot command to PLC"""
        try:
            # Calculate body motion from current to target pose (use dummy current pose for consistency)
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
            
            # Use the free gait method to send coordinated command
            success = self._send_free_gait_to_plc(body_motion, foot_positions, foot_flags)
            
            if success:
                rospy.loginfo("Sent coordinated pose and foot command to PLC")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"PLC coordinated movement command failed: {str(e)}")
            return False

    def setCmd(self, **kwargs) -> bool:
        """Set parameters on both PLC and dummy interface"""
        # Set parameters on dummy interface
        dummy_success = super().setCmd(**kwargs)
        
        # Set parameters on PLC if available
        plc_success = True
        if self.real_cmd_enabled and self.plc_connected:
            try:
                plc_success = self._send_params_to_plc(**kwargs)
            except Exception as e:
                rospy.logwarn(f"Error setting parameters on PLC: {str(e)}")
                plc_success = False
        
        return dummy_success and plc_success

    def _send_params_to_plc(self, **kwargs) -> bool:
        """Send parameters to PLC"""
        try:
            if not self._read_plc_parameters():
                return False
            
            # Time parameters
            if 'TA' in kwargs and self.cmdTime:
                self.cmdTime["TA"] = kwargs['TA']
            if 'TM' in kwargs and self.cmdTime:
                self.cmdTime["TM"] = kwargs['TM']
            if 'TD' in kwargs and self.cmdTime:
                self.cmdTime["TD"] = kwargs['TD']
            if 'TZ' in kwargs and self.cmdTime:
                self.cmdTime["TZ"] = kwargs['TZ']
            
            # Gait parameters
            if 'GaitMode' in kwargs and self.cmdGait:
                self.cmdGait["GaitMode"] = kwargs['GaitMode']
            if 'GaitDF' in kwargs and self.cmdGait:
                self.cmdGait["GaitDF"] = kwargs['GaitDF']
            if 'SwapHigh' in kwargs and self.cmdGait:
                self.cmdGait["SwapHigh"] = kwargs['SwapHigh']
            if 'ForceMode' in kwargs and self.cmdGait:
                self.cmdGait["ForceMode"] = kwargs['ForceMode']
            
            # Write parameters to PLC
            if self.cmdTime and self.symbol_Cmd_Time:
                self.symbol_Cmd_Time.write(self.cmdTime)
            if self.cmdGait and self.symbol_Cmd_Gait:
                self.symbol_Cmd_Gait.write(self.cmdGait)
            
            rospy.loginfo(f"PLC parameters updated: {kwargs}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to set PLC parameters: {str(e)}")
            return False

    def stop_movement(self) -> bool:
        """Stop movement on both PLC and dummy interface"""
        # Stop dummy movement
        dummy_success = super().stop_movement()
        
        # Stop PLC if available
        plc_success = True
        if self.real_cmd_enabled and self.plc_connected:
            try:
                if self.symbol_CtrlCmd:
                    self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)
                    rospy.loginfo("PLC movement stopped")
            except Exception as e:
                rospy.logwarn(f"Error stopping PLC movement: {str(e)}")
                plc_success = False
        
        return dummy_success and plc_success

    # Utility methods for debugging and control
    
    def enable_real_commands(self) -> bool:
        """Enable sending commands to real robot PLC"""
        if not self.plc_connected and PLC_AVAILABLE:
            self._init_plc_connection()
        
        self.real_cmd_enabled = True and self.plc_connected
        rospy.loginfo(f"Real robot PLC commands {'enabled' if self.real_cmd_enabled else 'disabled (connection failed)'}")
        return self.real_cmd_enabled

    def disable_real_commands(self):
        """Disable sending commands to real robot PLC (dummy mode only)"""
        self.real_cmd_enabled = False
        rospy.loginfo("Real robot PLC commands disabled - dummy mode only")

    def get_real_robot_status(self) -> dict:
        """Get status information about real robot PLC connection"""
        return {
            'plc_modules_available': PLC_AVAILABLE,
            'plc_connected': self.plc_connected,
            'cpp_connected': self.cpp_connected,
            'real_cmd_enabled': self.real_cmd_enabled,
            'plc_ip': self.plc_ip,
            'plc_enabled': self._is_plc_enabled() if self.plc_connected else False,
        }

    def robot_is_moving(self) -> bool:
        """
        Override to always use dummy interface feedback for consistency.
        Real robot movement status is not used for feedback.
        """
        if PLC_AVAILABLE:
            return not self.symbol_ReqFlag.value
        else:
            return super().robot_is_moving()