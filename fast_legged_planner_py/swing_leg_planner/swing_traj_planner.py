'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-16 21:49:12
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-12-01 20:51:39
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//swing_traj_planner.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from ..perception_interface.gridmap_interface_ros import GridMap_Interface
from .traj_gen.traj_gen import HermiteSpline
from .traj_opt.traj_opt import HermiteOptProb, UniBSplineOptProb, Legged_UniBSplineOptProb, RRTBSplineOptProb, RRTCfg_OptProb
from .cost.cost import CostCollection, KinematicCost, CollisionCost
from .collision.collision import HITLeg_Collision_Model
from ..utils.benchmark.benchmark import do_cprofile


class SwingTrajPlanner(object):
    def __init__(self, map_interface: GridMap_Interface, robot_interface=None):
        self.map_interface = map_interface
        self.robot_interface = robot_interface
        pass

    @do_cprofile(save=True)
    def opt_traj(self, default_traj, torso_traj=None, leg_index=None, maxiter=20, ret=False):
        """Get the swing trajectory
        :param default_traj: default swing trajectory (with time) (param by reference)
        :param torso_traj: torso trajectory (with time) (FIXME: class for function?)
        """
        # Init
        spline = default_traj
        collmodel = HITLeg_Collision_Model(
            self.robot_interface, self.map_interface, leg_index)

        # Hermite Opt
        # costs = CostCollection([
        #     KinematicCost(spline, 0.1),
        #     CollisionCost(spline, self.map_interface, 10)
        # ])
        # HermiteOptProb(spline, costs, None,
        #                spline.get()).optimize(maxiter=maxiter)

        # Uniform B-Spline
        # UniBSplineOptProb(spline, self.map_interface).optimize(maxiter=maxiter, disp=True)
        # Uniform B-Spline with Leg Collision
        Legged_UniBSplineOptProb(spline, torso_traj, self.map_interface, collmodel).optimize(
            maxiter=maxiter, disp=True)



        # RRT BSpline Search
        # RRTBSplineOptProb(spline, self.map_interface,
        #                   z_margin=0.5, obs_clearance=0.05,
        #                   end_ignore_dia=0.08).optimize(Q=np.array([[0.1, 4]]), max_samples=1024)
        # RRT Cfg Search (Need to enable ret)
        # spline = RRTCfg_OptProb(spline, torso_traj, leg_index, self.map_interface, self.robot_interface,
        #                         end_ignore_dia=0.07).optimize(Q=np.array([[0.05, 4]]), max_samples=1024)

        if ret:
            return spline
