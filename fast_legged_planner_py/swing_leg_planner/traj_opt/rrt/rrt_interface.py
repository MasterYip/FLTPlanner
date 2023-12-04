'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-27 13:47:50
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-12-04 12:07:09
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_opt//rrt//rrt_interface.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
from tkinter import E
from ...collision.collision import HITLeg_Collision_Model
from ....third_party.rrt_algorithms.src.search_space.search_space import SearchSpace
import numpy as np
from typing_extensions import override


################################################
# Ompl interface
################################################
from ompl import base
from ompl import geometric


class OMPL_GridmapSearchSpace(object):
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
        """
        # z margin for search space
        dimension_lengths = np.array(map_interface.get_sdfrange())
        dimension_lengths[-1][1] += z_margin
        self.map_interface = map_interface

        # Border clearance
        self.obs_clearance = obs_clearance
        # Start and goal obstacles ignoring
        self.end_ignore_dia = end_ignore_dia
        # Start and goal
        self.start = start
        self.goal = goal

        # OMPL space
        # Create an instance of the state space
        self.space = base.RealVectorStateSpace(3)
        # Set bounds for the state space
        bounds = base.RealVectorBounds(3)
        # FIXME:
        for i in range(3):
            bounds.setLow(i, dimension_lengths[i][0])
            bounds.setHigh(i, dimension_lengths[i][1])
        self.space.setBounds(bounds)

        # Create an instance of the space information
        self.space_instance = base.SpaceInformation(self.space)
        self.space_instance.setStateValidityChecker(
            base.StateValidityCheckerFn(self.obstacle_free))

    def get_space_instance(self):
        return self.space_instance

    def get_space_info(self):
        return self.space

    def obstacle_free(self, x, use_sdf=True):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :param use_sdf: use GridMap_SDF or GridMap
        :return: True if not inside an obstacle, False otherwise
        """
        x = np.array([x[0], x[1], x[2]])
        try:
            if use_sdf:
                ret = self.map_interface.sdf_value(
                    np.array(x)) > self.obs_clearance
            else:
                # FIXME: This do not support ground & ceiling map
                ret = x[-1] > (self.map_interface.value(
                    np.array(x[:2])) + self.obs_clearance)
        except Exception:
            print("Out of range:", x)
            ret = True  # Because the barrier height may occupy the entire height range, blocking the robot from moving
        # Start and goal obstacles ignoring
        if np.linalg.norm(np.array(x) - self.start) < self.end_ignore_dia\
                or np.linalg.norm(np.array(x) - self.goal) < self.end_ignore_dia:
            ret = True
        return ret


################################################
# rrt_algorithm interface
################################################


class HITSpiderCfg_SearchSpace(SearchSpace):
    def __init__(self, robot_interface, map_interface,
                 torso_traj, leg_index, start, goal,
                 end_ignore_dia=0.05):
        """
        :param robot_interface: Robot_Interface
        :param map_interface: GridMap_Interface
        :param torso_traj: torso trajectory (t in [0, 1]) (FIXME: Temporarily a function)
        :param leg_index: leg index
        :param start: start position
        :param goal: goal position
        :param end_ignore_dia: start and goal obstacles ignoring (should be larger than obs_clearance)
        TODO: start/end ignore should be improved
        """
        dimension_lengths = np.concatenate(
            ([[0., 1.]], robot_interface.get_leg_cfg_space()))
        super().__init__(dimension_lengths, None)

        self.map_interface = map_interface
        self.robot_interface = robot_interface
        self.collmodel = HITLeg_Collision_Model(
            self.robot_interface, self.map_interface, leg_index)
        self.leg_index = leg_index
        # torso trajectory evaluation
        self.eval_torso_traj = torso_traj
        # Start and goal obstacles ignoring
        self.end_ignore_dia = end_ignore_dia
        # Start and goal (config space)
        self.start = start
        self.goal = goal

    @override
    def obstacle_free(self, x_cfg, use_sdf=True):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :param use_sdf: use GridMap_SDF or GridMap
        :return: True if not inside an obstacle, False otherwise
        """

        try:
            if use_sdf:
                ret = not self.collmodel.checkCollision(
                    q_leg=x_cfg[1:], pose_base=self.eval_torso_traj(x_cfg[0]))
            else:
                raise NotImplementedError("Not use_sdf is Not implemented")
        except:
            print("Out of range:", x_cfg)
            ret = True  # Because the barrier height may occupy the entire height range, blocking the robot from moving
        # Start and goal obstacles ignoring (NOTE this is 4 dimen sphere)
        if np.linalg.norm(np.array(x_cfg) - self.start) < self.end_ignore_dia\
                or np.linalg.norm(np.array(x_cfg) - self.goal) < self.end_ignore_dia:
            ret = True
        return ret


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
        dimension_lengths[-1][1] += z_margin
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
    def obstacle_free(self, x, use_sdf=True):
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
                # FIXME: This do not support ground & ceiling map
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
