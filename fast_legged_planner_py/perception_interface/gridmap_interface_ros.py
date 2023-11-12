#!/usr/bin/env python
# coding=utf-8

import rospy
from grid_map_msgs.msg import GridMap as GridMapMsg
from grid_map import GridMap
from grid_map import SignedDistanceField


class GridMap_Interface(object):
    def __init__(self, topic_name: str = "grid_map") -> None:
        self.sub = rospy.Subscriber(topic_name, GridMapMsg, self.callback)
        
        self.msg = GridMapMsg()
        self.grid_map = GridMap()
        self.sdf = None
        
        self.elevation_layer = "3D_feeling"

    def callback(self, msg):
        self.msg = msg

    def update(self):
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
            self.sdf = SignedDistanceField(self.grid_map, layer_name, min_height, max_height)
            return True
        except IndexError:
            rospy.logwarn("Layer %s not found!", layer_name)
            return False
    
    def sdf_value(self, position):
        if self.sdf is None:
            return None
        return self.sdf.value(position)
