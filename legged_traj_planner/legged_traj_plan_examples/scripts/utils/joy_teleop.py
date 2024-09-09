#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')
        # Default value
        cmd_topic = 'cmd_vel'
        joy_topic = 'joy'
        self.pub_rate = 10.0  # Adjust as needed
        self.max_linear_x = 0.4  # Adjust as needed
        self.max_linear_y = 0.2  # Adjust as needed
        self.max_angular_z = 0.4  # Adjust as needed
        # Ros param
        cmd_topic = rospy.get_param('~cmd_topic', cmd_topic)
        joy_topic = rospy.get_param('~joy_topic', joy_topic)
        self.pub_rate = rospy.get_param('~pub_rate', self.pub_rate)
        self.max_linear_x = rospy.get_param('~max_linear_x', self.max_linear_x)
        self.max_linear_y = rospy.get_param('~max_linear_y', self.max_linear_y)
        self.max_angular_z = rospy.get_param('~max_angular_z', self.max_angular_z)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.pub_rate), self.timer_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        self.twist_cmd = Twist()

    def joy_callback(self, joy_msg):
        self.twist_cmd.linear.x = self.max_linear_x * joy_msg.axes[1]  # Forward/backward control
        self.twist_cmd.linear.y = self.max_linear_y * joy_msg.axes[0]  # Left/right control
        self.twist_cmd.angular.z = self.max_angular_z * joy_msg.axes[3]  # Rotation control

    def timer_callback(self, event):
        self.cmd_vel_pub.publish(self.twist_cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    joy_teleop = JoyTeleop()
    joy_teleop.run()
