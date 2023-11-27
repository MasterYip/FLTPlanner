'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-27 13:47:50
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-27 17:38:43
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_opt//rrt//rrt_interface.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from typing_extensions import override
from ....third_party.rrt_algorithms.src.search_space.search_space import SearchSpace


class Gridmap_SearchSpace(SearchSpace):
    def __init__(self, map_interface):
        # FIXME: margin is needed
        z_margin = 1.0
        dimension_lengths = np.array(map_interface.get_sdfrange())
        dimension_lengths[-1] += z_margin
        self.map_interface = map_interface
        super().__init__(dimension_lengths, None)

        # Border clearance
        self.free_margin = 0.1
        
        # TODO: start and goal should be handled

    @override
    def obstacle_free(self, x, use_sdf=False):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :param use_sdf: use GridMap_SDF or GridMap
        :return: True if not inside an obstacle, False otherwise
        """

        try:
            if use_sdf:
                ret = self.map_interface.sdf_value(
                    np.array(x)) > self.free_margin
            else:
                ret = x[-1] > (self.map_interface.value(
                    np.array(x[:2])) + self.free_margin)
                if not ret:
                    print("x:", x[-1], "map:", self.map_interface.value(
                        np.array(x[:2])) + self.free_margin)
        except:
            print("Out of range:", x)
            ret = True  # Because the barrier height may occupy the entire height range, blocking the robot from moving
        return ret
