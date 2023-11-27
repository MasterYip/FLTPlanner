'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-27 13:47:50
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-27 22:14:14
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_opt//rrt//rrt_interface.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from typing_extensions import override
from ....third_party.rrt_algorithms.src.search_space.search_space import SearchSpace


class Gridmap_SearchSpace(SearchSpace):
    def __init__(self, map_interface, start, goal,
                 z_margin=1.0, obs_clearance=0.04,
                 end_ignore_dia=0.05):
        """
        :param map_interface: GridMap_Interface
        :param start: start position
        :param goal: goal position
        :param z_margin: z margin for search space
        :param obs_clearance: obstacle clearance
        :param end_ignore_dia: start and goal obstacles ignoring (should be larger than obs_clearance)
        TODO: start/end ignore should be improved
        """
        # z margin for search space
        dimension_lengths = np.array(map_interface.get_sdfrange())
        dimension_lengths[-1] += z_margin
        self.map_interface = map_interface
        super().__init__(dimension_lengths, None)

        # Border clearance
        self.obs_clearance = obs_clearance
        # Start and goal obstacles ignoring
        self.end_ignore_dia = end_ignore_dia
        # Start and goal
        self.start = start
        self.goal = goal

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
                    np.array(x)) > self.obs_clearance
            else:
                ret = x[-1] > (self.map_interface.value(
                    np.array(x[:2])) + self.obs_clearance)
        except:
            print("Out of range:", x)
            ret = True  # Because the barrier height may occupy the entire height range, blocking the robot from moving
        # Start and goal obstacles ignoring
        if np.linalg.norm(np.array(x) - self.start) < self.end_ignore_dia\
                or np.linalg.norm(np.array(x) - self.goal) < self.end_ignore_dia:
            ret = True
        return ret
