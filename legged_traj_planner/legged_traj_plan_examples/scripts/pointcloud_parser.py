#!/usr/bin/env python
# coding=utf-8

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2


def callback(data):
    # rospy.loginfo()
    pc_array = pc2.read_points_numpy(
        data, field_names=("x", "y", "z"), skip_nans=False)
    for pc in pc_array:
        print(len(pc))
    # print("shape:", pc_array.shape)
    # print(pc_array[0])


def listener():
    rospy.init_node('sub_point_cloud', anonymous=True)
    sub = rospy.Subscriber("/full_sdf", PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
