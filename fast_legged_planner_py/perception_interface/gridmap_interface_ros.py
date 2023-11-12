#!/usr/bin/env python
# coding=utf-8

import rospy
from grid_map_msgs.msg import GridMap as GridMapMsg
from grid_map import GridMap


class GridMap_Interface(object):
    def __init__(self, topic_name: str = "grid_map") -> None:
        self.sub = rospy.Subscriber(topic_name, GridMapMsg, self.callback)
        self.grid_map = GridMap()
        # self.grid_map = GridMap(['elevation', 'intensity', 'roughness'])
        # self.grid_map.setGeometry([1, 2], 0.5, [3, 10])
        self.msg = GridMapMsg()

    def callback(self, msg):
        self.msg = msg

    def update(self):
        self.grid_map = GridMap.from_msg(self.msg)


if __name__ == '__main__':
    try:
        rospy.init_node('GridMap_Interface_py', anonymous=True)
        gmI = GridMap_Interface()
        while (rospy.is_shutdown() is False):
            gmI.update()
            # print(gmI.grid_map.getLayers())
            try:
                # print(gmI.grid_map.get('3D_feeling'))
                print(gmI.grid_map.at('3D_feeling', [0, 0]))
            except:
                print("Layer not found!")
            rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
