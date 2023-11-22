'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-16 21:49:12
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-22 15:13:02
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//swing_traj_planner.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from ..perception_interface.gridmap_interface_ros import GridMap_Interface
from .traj_gen.traj_gen import HermiteSpline
from .traj_opt.traj_opt import HermiteOptProb
from .cost.cost import CostCollection, KinematicCost, CollisionCost


class SwingTrajPlanner(object):
    def __init__(self, map_interface: GridMap_Interface, robot_interface=None):
        self.map_interface = map_interface
        self.robot_interface = robot_interface
        pass

    def opt_traj(self, default_traj, torso_traj=None, maxiter=50, ret=False):
        """Get the swing trajectory
        :param default_traj: default swing trajectory (with time) (param by reference)
        :param torso_traj: torso trajectory (with time)
        """

        spline = default_traj
        costs = CostCollection([
            KinematicCost(spline, 0.1),
            CollisionCost(spline, self.map_interface, 10)
        ])
        HermiteOptProb(spline, costs, None,
                       spline.get()).optimize(maxiter=maxiter)
        if ret:
            return spline
