'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-17 11:44:52
LastEditors: RaymonYip-NUC11
LastEditTime: 2024-01-16 21:21:30
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/whole_body_planner/hit_spider_planner_ros.py
Description: This whole body planner supports ElSpider2 & ElSpider_Air
'''
#!/usr/bin/env python
# coding=utf-8

import numpy as np
import pinocchio as pin
from ..utils.data_structure import CircleQueue
from .whole_body_planner import WholeBodyPlanner
from ..swing_leg_planner.swing_traj_planner import SwingTrajPlanner
from ..robot_interface.hitspider_robotinterface_ros import FeetPos2PosList, XYZRPY2SE3
from fast_legged_planner.msg import hexapod_State


class MCTStateTransfer(object):
    """State transfer trajectory of HITSpider(state0 to state1)"""

    def __init__(self, state0: hexapod_State, state1: hexapod_State,
                 swing_traj_planner: SwingTrajPlanner) -> None:
        """
        :param state0: start state
        :param state1: end state
        :param swing_traj_planner: swing trajectory planner
        """
        self.swing_traj_planner = swing_traj_planner
        self.state0 = state0
        self.state1 = state1
        self.torso_traj = None
        '''Torso Trajectory'''
        # TODO:
        '''Swing Trajectory'''
        self.footpos_list0 = FeetPos2PosList(state0.feetPositionNow)
        self.footpos_list1 = FeetPos2PosList(state1.feetPositionNow)
        # Swing trajectory Cache
        self.swingtraj = [None for i in range(6)]
        # Trajectory optimization flag
        self.swingtraj_isopt = [False for i in range(6)]
        # Support leg is not needed
        self.swingtraj_isneeded = [
            self.state1.support_State_Now[i] == 0 for i in range(6)]
        # Default swing trajectory
        v_lift = 0.1
        h_lift = 0.1
        for i in range(6):
            if self.swingtraj_isneeded[i]:
                self.swingtraj[i] = self.swing_traj_planner.get_default_traj(
                    self.footpos_list0[i], self.footpos_list1[i], v_lift, h_lift)

    # Traj evaluation

    def eval_torso_traj(self, t):
        """Evaluate torso trajectory at time t
        Args:
            t (float): interpolation time(not real time)
        Returns:
            pin.SE3: torso pose
        """
        pose0 = XYZRPY2SE3(self.state0.base_Pose_Now)
        pose1 = XYZRPY2SE3(self.state1.base_Pose_Now)
        err = pin.log(pose0.actInv(pose1))  # FIXME
        odom_interp = pose0.act(pin.exp(err*t))
        return odom_interp

    def eval_foot_traj(self, t, auto_opt=True):
        """
        Evaluate foot position between state0 and state1
        :param t: normalized interpolation time (from 0 to 1)
        """
        footend_interp = []
        for i in range(6):
            if self.swingtraj_isneeded[i]:
                # TODO: Note: traj should be optimized parallelly, this is just a temporary solution
                if not self.swingtraj_isopt[i] and auto_opt:
                    self.opt_swing_traj(i)
                footend_interp.append(
                    self.swingtraj[i].evaluate(t, normalized=True))
            else:
                footend_interp.append(self.footpos_list0[i])
        return footend_interp

    # Swing traj optimization
    def opt_swing_traj(self, index):
        """Optimize swing trajectory
        Args:
            index (int): index of swing leg
        """
        if not self.opt_check(index):
            self.swingtraj[index] = self.swing_traj_planner.opt_traj(
                self.swingtraj[index], self.eval_torso_traj, index)
            self.swingtraj_isopt[index] = True

    def opt_check(self, index=None):
        """Check if (all) swing trajectory is optimized
        :return: False if need optimization , True if (all) swing trajectory is optimized (or don't need optimization)
        """
        if index is not None:
            return self.swingtraj_isopt[index] or not self.swingtraj_isneeded[index]
        else:
            return all([self.swingtraj_isopt[i] or not self.swingtraj_isneeded[i] for i in range(6)])


class HITSpiderWholeBodyPlanner(WholeBodyPlanner):
    def __init__(self, map_interface, robot_interface) -> None:
        self.state_trajs = CircleQueue(100)
        self.swing_traj_planner = SwingTrajPlanner(
            map_interface, robot_interface)

    def enqueue_MCTsolution(self, state0, state1):
        return self.state_trajs.enqueue(MCTStateTransfer(state0, state1, self.swing_traj_planner))

    def dequeue_MCTsolution(self):
        return self.state_trajs.dequeue()

    def get_state_traj(self, index):
        return self.state_trajs.at(index)

    def get_state_traj_length(self):
        return self.state_trajs.get_length()

    # Visualization
    def get_foot_traj(self, t, point_num=80, delta=0.05):
        """Get foot trajectory
        :param t: normalized interpolation time
        :param point_num: number of points in the trajectory
        :param delta: time interval between two points
        :return: foot trajectory list (default_traj_list, opt_traj_list)
        """
        # For TrajViz
        default_traj_list = [[] for _ in range(6)]
        opt_traj_list = [[] for _ in range(6)]
        index = 0
        while (point_num > 0 and self.state_trajs.is_valid(index)):
            state_traj = self.state_trajs.at(index)
            while (t < 1 and point_num > 0):
                foot_pos_list = state_traj.eval_foot_traj(t, False)
                for j in range(6):
                    # FIXME: It is not recommanded to use private var
                    if state_traj.opt_check(j) and state_traj.swingtraj_isneeded[j]:
                        opt_traj_list[j].append(foot_pos_list[j])
                    else:
                        default_traj_list[j].append(foot_pos_list[j])
                point_num -= 1
                t += delta
            index += 1
            t = 0
        return default_traj_list, opt_traj_list
