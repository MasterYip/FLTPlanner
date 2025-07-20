#!/usr/bin/env python3

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler

# Import the hexapod interface
from hexapod201_interface import DummyHexapod201Interface


def test_dummy_interface():
    """Test the dummy hexapod interface"""
    print("Testing Dummy Hexapod Interface...")
    
    # Initialize ROS node
    rospy.init_node('test_hexapod_interface', anonymous=True)
    
    # Create dummy interface
    hexapod = DummyHexapod201Interface("test_dummy_interface")
    
    # Wait for initialization
    rospy.sleep(2.0)
    
    # Test 1: Check initial pose
    print("Test 1: Checking initial pose...")
    current_pose = hexapod.get_current_pose()
    print(f"Initial pose: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
    
    # Test 2: Set movement parameters
    print("Test 2: Setting movement parameters...")
    success = hexapod.setCmd(movement_speed=0.2, rotation_speed=1.0)
    print(f"Parameter setting: {'SUCCESS' if success else 'FAILED'}")
    
    # Test 3: Send velocity command
    print("Test 3: Sending velocity command...")
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Send forward velocity
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.1
    cmd_vel_pub.publish(cmd_vel)
    
    # Wait for movement
    rospy.sleep(3.0)
    
    # Check new pose
    current_pose = hexapod.get_current_pose()
    print(f"After velocity command: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
    
    # Test 4: Send pose command
    print("Test 4: Sending pose command...")
    pose_cmd_pub = rospy.Publisher('/hexapod/pose_cmd', PoseStamped, queue_size=10)
    
    # Create pose message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "odom"
    pose_msg.pose.position.x = 1.0
    pose_msg.pose.position.y = 0.5
    pose_msg.pose.position.z = 0.0
    
    # Set orientation (90 degree turn)
    quat = quaternion_from_euler(0.0, 0.0, np.pi/2)
    pose_msg.pose.orientation.w = quat[0]
    pose_msg.pose.orientation.x = quat[1]
    pose_msg.pose.orientation.y = quat[2]
    pose_msg.pose.orientation.z = quat[3]
    
    pose_cmd_pub.publish(pose_msg)
    
    # Wait for movement
    rospy.sleep(5.0)
    
    # Check final pose
    current_pose = hexapod.get_current_pose()
    print(f"After pose command: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f})")
    
    # Test 5: Stop movement
    print("Test 5: Stopping movement...")
    success = hexapod.stop_movement()
    print(f"Stop movement: {'SUCCESS' if success else 'FAILED'}")
    
    # Test 6: Reset integration
    print("Test 6: Resetting integration...")
    hexapod.reset_integration()
    print("Integration reset completed")
    
    print("All tests completed!")
    return True


def test_visualization():
    """Test the visualization system"""
    print("Testing Visualization System...")
    
    # Initialize ROS node
    rospy.init_node('test_visualization', anonymous=True)
    # Import and test visualizer
    try:
        from ros_visualizer import ROSVisualizer, VisStyle
        
        # Create visualizer
        visualizer = ROSVisualizer("odom", "test_visualization")
        
        # Test 1: Create a cube
        print("Test 1: Creating cube...")
        position = np.array([0.0, 0.0, 0.0])
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        visualizer.vis_cube(position, quat)
        rospy.sleep(1.0)
        
        # Test 2: Create a sphere
        print("Test 2: Creating sphere...")
        sphere_pos = np.array([1.0, 0.0, 0.0])
        visualizer.vis_sphere(sphere_pos, 0.1)
        rospy.sleep(1.0)
        
        # Test 3: Create an arrow
        print("Test 3: Creating arrow...")
        start = np.array([0.0, 0.0, 0.0])
        end = np.array([0.5, 0.5, 0.0])
        visualizer.vis_arrow(start, end)
        rospy.sleep(1.0)
        
        # Test 4: Create a curve
        print("Test 4: Creating curve...")
        curve_points = [
            np.array([0.0, 0.0, 0.0]),
            np.array([0.5, 0.5, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([1.5, -0.5, 0.0])
        ]
        visualizer.vis_curve(curve_points)
        rospy.sleep(1.0)
        
        # Test 5: Clear all
        print("Test 5: Clearing all markers...")
        visualizer.del_all()
        rospy.sleep(1.0)
        
        print("Visualization tests completed!")
        return True
        
    except ImportError as e:
        print(f"Failed to import visualizer: {e}")
        return False
    except Exception as e:
        print(f"Visualization test failed: {e}")
        return False


def main():
    """Main test function"""
    print("=== Hexapod Interface Test Suite ===")
    
    try:
        # Test visualization first
        vis_success = test_visualization()
        
        # Test dummy interface
        interface_success = test_dummy_interface()
        
        print("\n=== Test Results ===")
        print(f"Visualization: {'PASS' if vis_success else 'FAIL'}")
        print(f"Interface: {'PASS' if interface_success else 'FAIL'}")
        
        if vis_success and interface_success:
            print("All tests PASSED!")
        else:
            print("Some tests FAILED!")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    
    print("Test suite completed.")


if __name__ == "__main__":
    main() 