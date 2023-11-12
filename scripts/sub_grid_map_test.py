#!/usr/bin/env python
# coding=utf-8

import rospy
from grid_map_msgs.msg import GridMap as GridMapMsg
from grid_map import GridMap


def callback(data):
    # rospy.loginfo()
    print(data.info.header)


def listener():

    rospy.init_node('GridMap_python_test', anonymous=True)
    sub = rospy.Subscriber("/grid_map", GridMapMsg, callback)

    rospy.spin()


if __name__ == '__main__':

    a = GridMap(['elevation', 'intensity', 'roughness'])
    a.setGeometry([1, 2], 0.5, [3, 10])
    print(a)
    # <1.00x2.00x0.50 grid on ['elevation', 'intensity', 'roughness'] at [ 3. 10.]>

    a['elevation'][0, 1:2] = 1
    print(a['elevation'])
    # [[nan,  1., nan, nan],
    #  [nan, nan, nan, nan]]
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
