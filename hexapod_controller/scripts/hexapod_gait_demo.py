#!/usr/bin/env python3

import rospy
import numpy as np
import time
import argparse
from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import the hexapod interface
from .hexapod201_interface import Hexapod201Interface, DummyHexapod201Interface


class HexapodGaitDemo:
    """Demo class for hexapod gait control and body movement"""
    
    def __init__(self, use_dummy: bool = True, plc_ip: str = "5.157.100.214.1.1"):
        """
        Initialize the demo
        
        Args:
            use_dummy: Whether to use dummy interface for simulation
            plc_ip: PLC IP address for real interface
        """
        self.use_dummy = use_dummy
        
        # Initialize ROS node
        rospy.init_node('hexapod_gait_demo', anonymous=True)
        
        # Initialize hexapod interface
        if use_dummy:
            self.hexapod = DummyHexapod201Interface("dummy_hexapod_demo")
            rospy.loginfo("Using dummy hexapod interface for simulation")
        else:
            self.hexapod = Hexapod201Interface("real_hexapod_demo", plc_ip)
            rospy.loginfo("Using real hexapod interface with PLC")
        
        # Publishers for demo commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_cmd_pub = rospy.Publisher('/hexapod/pose_cmd', PoseStamped, queue_size=10)
        
        # Demo parameters
        self.demo_running = False
        self.demo_thread = None
        
        rospy.loginfo("Hexapod gait demo initialized")
    
    def start_velocity_demo(self):
        """Start velocity command demo"""
        if self.demo_running:
            rospy.logwarn("Demo already running")
            return
        
        self.demo_running = True
        rospy.loginfo("Starting velocity command demo")
        
        # Create velocity commands for different movements
        movements = [
            # Forward movement
            {'linear': [0.1, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 5.0},
            # Stop
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 2.0},
            # Turn left
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.5], 'duration': 3.0},
            # Stop
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 2.0},
            # Sideways movement
            {'linear': [0.0, 0.1, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 5.0},
            # Stop
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 2.0},
            # Turn right
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, -0.5], 'duration': 3.0},
            # Stop
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'duration': 2.0},
        ]
        
        rate = rospy.Rate(10)  # 10 Hz
        
        for i, movement in enumerate(movements):
            if not self.demo_running:
                break
            
            rospy.loginfo(f"Movement {i+1}: {movement}")
            
            # Send velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = movement['linear'][0]
            cmd_vel.linear.y = movement['linear'][1]
            cmd_vel.linear.z = movement['linear'][2]
            cmd_vel.angular.x = movement['angular'][0]
            cmd_vel.angular.y = movement['angular'][1]
            cmd_vel.angular.z = movement['angular'][2]
            
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Wait for duration
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < movement['duration']:
                if not self.demo_running:
                    break
                rate.sleep()
        
        # Stop at the end
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.demo_running = False
        rospy.loginfo("Velocity demo completed")
    
    def start_pose_demo(self):
        """Start pose command demo"""
        if self.demo_running:
            rospy.logwarn("Demo already running")
            return
        
        self.demo_running = True
        rospy.loginfo("Starting pose command demo")
        
        # Create pose commands for different positions
        poses = [
            # Move forward
            {'position': [1.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
            # Move to the right
            {'position': [1.0, 1.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
            # Turn 90 degrees
            {'position': [1.0, 1.0, 0.0], 'orientation': [0.0, 0.0, np.pi/2]},
            # Move forward again
            {'position': [2.0, 1.0, 0.0], 'orientation': [0.0, 0.0, np.pi/2]},
            # Turn back
            {'position': [2.0, 1.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
            # Return to origin
            {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]},
        ]
        
        for i, pose_data in enumerate(poses):
            if not self.demo_running:
                break
            
            rospy.loginfo(f"Pose {i+1}: {pose_data}")
            
            # Create pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            
            pose_msg.pose.position.x = pose_data['position'][0]
            pose_msg.pose.position.y = pose_data['position'][1]
            pose_msg.pose.position.z = pose_data['position'][2]
            
            # Convert Euler angles to quaternion
            quat = quaternion_from_euler(
                pose_data['orientation'][0],
                pose_data['orientation'][1],
                pose_data['orientation'][2]
            )
            pose_msg.pose.orientation.w = quat[0]
            pose_msg.pose.orientation.x = quat[1]
            pose_msg.pose.orientation.y = quat[2]
            pose_msg.pose.orientation.z = quat[3]
            
            # Send pose command
            self.pose_cmd_pub.publish(pose_msg)
            
            # Wait for movement to complete (longer for real robot)
            wait_time = 10.0 if not self.use_dummy else 5.0
            rospy.sleep(wait_time)
        
        self.demo_running = False
        rospy.loginfo("Pose demo completed")
    
    def start_circle_demo(self):
        """Start circular movement demo"""
        if self.demo_running:
            rospy.logwarn("Demo already running")
            return
        
        self.demo_running = True
        rospy.loginfo("Starting circular movement demo")
        
        # Circle parameters
        radius = 1.0  # meters
        angular_velocity = 0.2  # rad/s
        duration = 20.0  # seconds
        
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration and self.demo_running:
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            angle = angular_velocity * elapsed_time
            
            # Calculate position on circle
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            
            # Create pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            
            # Orient towards movement direction
            quat = quaternion_from_euler(0.0, 0.0, angle + np.pi/2)
            pose_msg.pose.orientation.w = quat[0]
            pose_msg.pose.orientation.x = quat[1]
            pose_msg.pose.orientation.y = quat[2]
            pose_msg.pose.orientation.z = quat[3]
            
            # Send pose command
            self.pose_cmd_pub.publish(pose_msg)
            
            rate.sleep()
        
        # Return to origin
        if self.demo_running:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            
            self.pose_cmd_pub.publish(pose_msg)
            rospy.sleep(5.0)
        
        self.demo_running = False
        rospy.loginfo("Circle demo completed")
    
    def start_gait_parameter_demo(self):
        """Start demo with different gait parameters"""
        if self.demo_running:
            rospy.logwarn("Demo already running")
            return
        
        if self.use_dummy:
            rospy.logwarn("Gait parameter demo only available for real hexapod")
            return
        
        self.demo_running = True
        rospy.loginfo("Starting gait parameter demo")
        
        # Different gait configurations
        gait_configs = [
            {'GaitMode': 1, 'GaitDF': 0.5, 'SwapHigh': 80.0, 'name': 'Synchronous gait'},
            {'GaitMode': 2, 'GaitDF': 0.67, 'SwapHigh': 100.0, 'name': 'Tripod gait'},
            {'GaitMode': 3, 'GaitDF': 0.75, 'SwapHigh': 60.0, 'name': 'Wave gait'},
        ]
        
        for i, config in enumerate(gait_configs):
            if not self.demo_running:
                break
            
            rospy.loginfo(f"Gait config {i+1}: {config['name']}")
            
            # Set gait parameters
            self.hexapod.setCmd(**config)
            
            # Move forward with this gait
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = 0.5
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            
            self.pose_cmd_pub.publish(pose_msg)
            rospy.sleep(8.0)
            
            # Return to origin
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            self.pose_cmd_pub.publish(pose_msg)
            rospy.sleep(8.0)
        
        self.demo_running = False
        rospy.loginfo("Gait parameter demo completed")
    
    def stop_demo(self):
        """Stop current demo"""
        self.demo_running = False
        if hasattr(self.hexapod, 'stop_movement'):
            self.hexapod.stop_movement()
        
        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        rospy.loginfo("Demo stopped")
    
    def print_status(self):
        """Print current hexapod status"""
        current_pose = self.hexapod.get_current_pose()
        target_pose = self.hexapod.get_target_pose()
        
        rospy.loginfo("=== Hexapod Status ===")
        rospy.loginfo(f"Current Position: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
        rospy.loginfo(f"Target Position: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        rospy.loginfo(f"Is Moving: {self.hexapod.is_moving}")
        rospy.loginfo(f"Demo Running: {self.demo_running}")
        rospy.loginfo("=====================")
    
    def run_interactive_demo(self):
        """Run interactive demo with user input"""
        rospy.loginfo("=== Hexapod Gait Demo ===")
        rospy.loginfo("Available commands:")
        rospy.loginfo("1 - Velocity demo")
        rospy.loginfo("2 - Pose demo")
        rospy.loginfo("3 - Circle demo")
        if not self.use_dummy:
            rospy.loginfo("4 - Gait parameter demo")
        rospy.loginfo("s - Stop demo")
        rospy.loginfo("p - Print status")
        rospy.loginfo("q - Quit")
        rospy.loginfo("========================")
        
        while not rospy.is_shutdown():
            try:
                command = input("Enter command: ").strip().lower()
                
                if command == '1':
                    self.start_velocity_demo()
                elif command == '2':
                    self.start_pose_demo()
                elif command == '3':
                    self.start_circle_demo()
                elif command == '4' and not self.use_dummy:
                    self.start_gait_parameter_demo()
                elif command == 's':
                    self.stop_demo()
                elif command == 'p':
                    self.print_status()
                elif command == 'q':
                    break
                else:
                    rospy.logwarn("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Error in interactive demo: {str(e)}")
        
        self.stop_demo()
        rospy.loginfo("Interactive demo ended")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Hexapod Gait Demo')
    parser.add_argument('--dummy', action='store_true', help='Use dummy interface for simulation')
    parser.add_argument('--plc_ip', type=str, default='5.157.100.214.1.1', help='PLC IP address')
    parser.add_argument('--demo', type=str, choices=['velocity', 'pose', 'circle', 'gait', 'interactive'], 
                       default='interactive', help='Demo type to run')
    
    args = parser.parse_args()
    
    try:
        # Create demo instance
        demo = HexapodGaitDemo(use_dummy=args.dummy, plc_ip=args.plc_ip)
        
        # Wait a bit for initialization
        rospy.sleep(2.0)
        
        # Run selected demo
        if args.demo == 'velocity':
            demo.start_velocity_demo()
        elif args.demo == 'pose':
            demo.start_pose_demo()
        elif args.demo == 'circle':
            demo.start_circle_demo()
        elif args.demo == 'gait' and not args.dummy:
            demo.start_gait_parameter_demo()
        elif args.demo == 'interactive':
            demo.run_interactive_demo()
        else:
            rospy.logwarn("Invalid demo type or not available for dummy interface")
        
        rospy.loginfo("Demo completed")
        
    except KeyboardInterrupt:
        rospy.loginfo("Demo interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in demo: {str(e)}")
    finally:
        if 'demo' in locals():
            demo.stop_demo()


if __name__ == "__main__":
    main() 