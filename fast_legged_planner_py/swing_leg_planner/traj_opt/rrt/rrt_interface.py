'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-27 13:47:50
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-27 14:01:24
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_opt//rrt//rrt_interface.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8

from ....third_party.rrt_algorithms.src.search_space.search_space import SearchSpace


class HIT_Spider_SearchSpace(SearchSpace):
    def __init__(self, dimension_lengths):
        super().__init__(dimension_lengths, None)
