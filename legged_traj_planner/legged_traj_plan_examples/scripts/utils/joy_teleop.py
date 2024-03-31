#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop:
    def __init__(self, cmd_topic='cmd_vel', joy_topic='joy'):
        rospy.init_node('joy_teleop')
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        self.max_linear_x = 1.0  # Adjust as needed
        self.max_linear_y = 0.1  # Adjust as needed

    def joy_callback(self, joy_msg):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.max_linear_x * joy_msg.axes[1]  # Forward/backward control
        twist_cmd.linear.y = self.max_linear_y * joy_msg.axes[0]  # Left/right control
        self.cmd_vel_pub.publish(twist_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    cmd_topic = rospy.get_param('~cmd_topic', 'cmd_vel')
    joy_topic = rospy.get_param('~joy_topic', 'joy')
    joy_teleop = JoyTeleop(cmd_topic, joy_topic)
    joy_teleop.run()
