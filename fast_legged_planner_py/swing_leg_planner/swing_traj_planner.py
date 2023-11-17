'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-16 21:49:12
LastEditors: NUC12
LastEditTime: 2023-11-17 11:43:30
FilePath: \Fast-Legged-Planner-Test\fast_legged_planner_py\swing_leg_planner\swing_traj_planner.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from ..perception_interface.gridmap_interface_ros import GridMap_Interface
from .traj_gen.traj_gen import HermiteSpline
from .traj_opt.traj_opt import TrajOptProblem
from .cost.cost import CostCollection, KinematicCost, CollisionCost


class SwingTrajPlanner(object):
    def __init__(self, map_interface: GridMap_Interface, robot_interface):
        self.map_interface = map_interface
        self.robot_interface = robot_interface
        pass

    def opt_traj(self, default_traj, torso_traj=None, ret=False):
        """Get the swing trajectory
        :param default_traj: default swing trajectory (with time) (param by reference)
        :param torso_traj: torso trajectory (with time)
        """

        spline = default_traj
        costs = CostCollection([
            KinematicCost(spline),
            CollisionCost(spline, self.map_interface)
        ])
        TrajOptProblem(spline, costs, None,
                       spline.knots).optimize()
        if ret:
            return spline
