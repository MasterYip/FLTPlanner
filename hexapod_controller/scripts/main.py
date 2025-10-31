from hexapod201_interface import *

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
        

if __name__ == "__main__":
    main()
    # test_interface()
    # test_pose_with_feet(0)
    # test_pose_with_feet(1)