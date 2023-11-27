#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from grid_map_msgs.msg import GridMap as GridMapMsg
from grid_map import GridMap
from grid_map import SignedDistanceField


class GridMap_Interface(object):
    def __init__(self, topic_name: str = "grid_map") -> None:
        self.sub = rospy.Subscriber(topic_name, GridMapMsg, self.callback)

        self.msg = GridMapMsg()
        self.grid_map = GridMap()
        self.sdf = None
        self.sdf_range = None
        # FIXME
        self.elevation_layer = rospy.get_param(
            "elevation_layer", "elevation")

    def callback(self, msg):
        self.msg = msg
        if self.sdf is None:
            self.update()

    def update(self, block=True):
        while self.msg.data == []:
            rospy.logwarn("GridMap_Interface - GridMap is not subscribed!")
            rospy.sleep(0.5)
        self.update_gridmap()
        self.update_sdf(self.elevation_layer)

    def update_gridmap(self):
        self.grid_map = GridMap.from_msg(self.msg)

    def update_sdf(self, layer_name: str, min_height=None, max_height=None):
        try:
            elevationData = self.grid_map.get(layer_name)
            if min_height is None:
                min_height = elevationData.min()
            if max_height is None:
                max_height = elevationData.max()
            self.sdf = SignedDistanceField(
                self.grid_map, layer_name, min_height, max_height)
            range = self.get_range()
            self.sdf_range = [(-0.5*range[0], 0.5*range[0]),
                              (-0.5*range[1], 0.5*range[1]),
                              (min_height, max_height)]
            # m = self.grid_map.getSize()-1
            # pos1 = np.zeros(2, dtype=np.float64)
            # pos2 = np.zeros(2, dtype=np.float64)
            # self.grid_map.getIndex(index=np.array([0, 0]), position=pos1)
            # self.grid_map.getPosition(index=np.array([5, 30]), position=pos2)
            # # print(self.grid_map.getPosition(np.array([0, 0])))
            # self.sdf_range = [(pos1[0], pos2[0]),
            #                   (pos1[1], pos2[1]),
            #                   (min_height, max_height)]
            return True
        except IndexError:
            rospy.logwarn("Layer %s not found!", layer_name)
            return False

    def value(self, position):
        return self.grid_map.atPosition(self.elevation_layer, position)

    def sdf_value(self, position):
        if self.sdf is None:
            return None
        return self.sdf.value(position)

    def sdf_derivative(self, position):
        if self.sdf is None:
            return None
        return self.sdf.derivative(position)

    def get_range(self):
        return self.grid_map.getLength()

    def get_sdfrange(self):
        return self.sdf_range
