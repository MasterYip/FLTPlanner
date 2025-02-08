#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios


class KeyTeleop:
    def __init__(self):
        rospy.init_node('key_teleop')
        # Default value
        cmd_topic = 'cmd_vel'
        self.pub_rate = 10.0  # Adjust as needed
        self.max_linear_x = 0.4  # Adjust as needed
        self.max_linear_y = 0.2  # Adjust as needed
        self.max_angular_z = 0.4  # Adjust as needed
        # Ros param
        cmd_topic = rospy.get_param('~cmd_topic', cmd_topic)
        self.pub_rate = rospy.get_param('~pub_rate', self.pub_rate)
        self.max_linear_x = rospy.get_param('~max_linear_x', self.max_linear_x)
        self.max_linear_y = rospy.get_param('~max_linear_y', self.max_linear_y)
        self.max_angular_z = rospy.get_param('~max_angular_z', self.max_angular_z)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.pub_rate), self.timer_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.twist_cmd = Twist()
        
        # Save the old tty settings
        self.old_tty_settings = termios.tcgetattr(sys.stdin)


    def timer_callback(self, event):
        self.update_cmd()
        self.cmd_vel_pub.publish(self.twist_cmd)

    def run(self):
        rospy.spin()

    def read_key(self):
        tty.setcbreak(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            return key
        return None

    def reset_tty(self):
        # FIXME: input are not displayed on the screen, so reset the tty
        termios.tcsetattr(sys.stdin, termios.TCSANOW, self.old_tty_settings)

    def update_cmd(self):
        key = self.read_key()
        if key == 'w':
            self.twist_cmd.linear.x = self.max_linear_x
        elif key == 's':
            self.twist_cmd.linear.x = -self.max_linear_x
        elif key == 'a':
            self.twist_cmd.linear.y = self.max_linear_y
        elif key == 'd':
            self.twist_cmd.linear.y = -self.max_linear_y
        elif key == 'q':
            self.twist_cmd.angular.z = self.max_angular_z
        elif key == 'e':
            self.twist_cmd.angular.z = -self.max_angular_z
        # ctrl + c
        elif key == '\x03':
            self.reset_tty()
            rospy.signal_shutdown('shutdown')


if __name__ == '__main__':
    key_teleop = KeyTeleop()
    key_teleop.run()
