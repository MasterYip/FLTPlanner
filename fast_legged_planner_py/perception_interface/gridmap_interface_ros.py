#!/usr/bin/env python
# coding=utf-8

from traitlets import default
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
        # FIXME
        self.ground_layer = rospy.get_param(
            "elevation_layer", default="elevation")
        self.ceiling_layer = rospy.get_param(
            "ceiling_layer", default="ceiling")
        # SDF
        # sdf[0] - ground(default), sdf[1] - ceiling
        self.sdf = [None for _ in range(2)]
        self.sdf_range = [None for _ in range(2)]

    def callback(self, msg):
        self.msg = msg
        if self.sdf[0] is None:
            self.update()

    def update(self, block=True, sdf_margin=0.2):
        while self.msg.data == [] and block:
            rospy.logwarn("GridMap_Interface - GridMap is not subscribed!")
            rospy.sleep(0.5)
        self.update_gridmap()
        self.update_sdf(self.ground_layer, margin=sdf_margin)
        print(self.msg.layers)
        if self.ceiling_layer in self.msg.layers:
            self.update_sdf(self.ceiling_layer, index=1, margin=sdf_margin)

    def update_gridmap(self):
        self.grid_map = GridMap.from_msg(self.msg)

    def update_sdf(self, layer_name: str, index=0, min_height=None, max_height=None, margin=0.2):
        """
        :param layer_name: layer name of the elevation layer
        :param min_height: minimum height of the SDF
        :param max_height: maximum height of the SDF
        :param margin: margin of the SDF (When min/max_height is None)
        """
        try:
            elevationData = self.grid_map.get(layer_name)
            if min_height is None:
                min_height = elevationData.min()-margin
            if max_height is None:
                max_height = elevationData.max()+margin
            self.sdf[index] = SignedDistanceField(
                self.grid_map, layer_name, min_height, max_height)
            range = self.get_range()
            self.sdf_range[index] = [(-0.5*range[0], 0.5*range[0]),
                                     (-0.5*range[1], 0.5*range[1]),
                                     (min_height, max_height)]
            # m = self.grid_map.getSize()-1
            # pos1 = np.zeros(2, dtype=np.float64)
            # pos2 = np.zeros(2, dtype=np.float64)
            # self.grid_map.getIndex(index=np.array([0, 0]), position=pos1)
            # self.grid_map.getPosition(index=np.array([5, 30]), position=pos2)
            # # print(self.grid_map.getPosition(np.array([0, 0])))
            # self.sdf_range[index] = [(pos1[0], pos2[0]),
            #                   (pos1[1], pos2[1]),
            #                   (min_height, max_height)]
            return True
        except IndexError:
            rospy.logwarn("Layer %s not found!", layer_name)
            return False

    def value(self, position, layer_name=None):
        if layer_name is None:
            layer_name = self.ground_layer
        return self.grid_map.atPosition(layer_name, position)

    def sdf_value(self, position, index=0, mode="min"):
        if mode == "min":
            if self.sdf[0] is None:
                rospy.logwarn("SDF is not initialized!")
                return None
            elif self.sdf[1] is None:
                rospy.logwarn("Ceiling SDF is not initialized!")
                return self.sdf[0].value(position)
            else:
                return min([self.sdf[0].value(position), -self.sdf[1].value(position)])
        elif mode == "ground":
            if self.sdf[index] is None:
                rospy.logwarn("SDF is not initialized!")
                return None
            return self.sdf[index].value(position)
        else:
            raise ValueError("mode should be 'min' or 'ground'")

    def sdf_derivative(self, position, index=0):
        if self.sdf[index] is None:
            rospy.logwarn("SDF is not initialized!")
            return None
        return self.sdf[index].derivative(position)

    def get_range(self):
        return self.grid_map.getLength()

    def get_sdfrange(self, index=0):
        return self.sdf_range[index]
