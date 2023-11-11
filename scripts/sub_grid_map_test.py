#!/usr/bin/env python
# coding=utf-8

import rospy
from grid_map_msgs.msg import GridMap


def callback(data):
    # rospy.loginfo()
    print(data.info.header)


def listener():

    rospy.init_node('GridMap_python_test', anonymous=True)
    sub = rospy.Subscriber("/grid_map", GridMap, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
