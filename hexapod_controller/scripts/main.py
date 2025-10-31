#!/usr/bin/env python3
from hexapod201_interface import *
from hexapod201_dummy_interface import DummyHexapod201Interface


def main():
    """Main function to run hexapod interface"""
    # Initialize ROS node first
    rospy.init_node('hexapod201_interface', anonymous=True)
    
    # Get parameters from ROS parameter server
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

def test_pose_with_feet(gait2phase=0):
    """Test move_to_pose_with_feet method"""
    rospy.init_node('test_hexapod_pose_with_feet', anonymous=True)
    interface = Hexapod201Interface(node_name="hexapod201_interface", plc_ip="5.157.100.214.1.1")
    
    # Define target pose - move forward 0.2m and turn 30 degrees
    target_pose = Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.05  # Lift body slightly
    
    # Convert 30 degrees to radians and create quaternion
    yaw_angle = math.pi / 6  # 30 degrees in radians
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
        [-560, 1096.2, -405], # Foot 3 - step back slightly
        [760, -896.2, -405],  # Foot 4 - step forward
        [0, -1251.2, -405],   # Foot 5 - keep in place  
        [-560, -896.2, -405]  # Foot 6 - step back slightly
    ])
    
    # Define foot support flags (0=support, 1=swing)
    # Alternate pattern: feet 1, 3, 5 swing, feet 2, 4, 6 support
    if gait2phase==0:
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
        rospy.loginfo(f"Final pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        
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
    main()
    # test_interface()
    # test_pose_with_feet(0)
    # test_pose_with_feet(1)