'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-27 13:47:50
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-27 15:48:54
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
        dimension_lengths = np.array(map_interface.get_sdfrange())
        self.map_interface = map_interface
        super().__init__(dimension_lengths, None)

    @override
    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        # return self.map_interface.sdf_value(np.array(x)) > 0
        try:
            ret = self.map_interface.value(np.array(x[:2])) < x[-1]
        except:
            print("Out of range:", x)
            ret = False
        return ret
