'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-17 11:44:52
LastEditors: NUC12
LastEditTime: 2023-11-18 22:30:10
FilePath: \\Fast-Legged-Planner-Test\\fast_legged_planner_py\\whole_body_planner\\hit_spider_planner_ros.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from abc import abstractmethod, ABCMeta
import pinocchio as pin
from ..utils.data_structure import CircleQueue
from ..swing_leg_planner.traj_gen.traj_gen import HermiteSpline
from .whole_body_planner import WholeBodyPlanner
from ..swing_leg_planner.swing_traj_planner import SwingTrajPlanner
from ..swing_leg_planner.traj_gen.traj_gen import linear_evaluate

from ..robot_interface.hitspider_robotinterface_ros import HITSpider_RobotInterface_ROS, \
    FeetPos2PosList, XYZRPY2SE3, point_SE3Act

from fast_legged_planner.msg import hexapod_State


class HITSpiderStateTraj(object):
    """State transfer trajectory of HITSpider(state0 to state1)"""

    def __init__(self, state0: hexapod_State, state1: hexapod_State, swing_traj_planner: SwingTrajPlanner) -> None:
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
        v = np.array([0, 0, 0.8])
        for i in range(6):
            if self.swingtraj_isneeded[i]:
                self.swingtraj[i] = HermiteSpline(
                    np.array([self.footpos_list0[i], v, self.footpos_list1[i], -v]))

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
                footend_interp.append(self.swingtraj[i].evaluate_normalized(t))
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
            self.swing_traj_planner.opt_traj(self.swingtraj[index])
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
        return self.state_trajs.enqueue(HITSpiderStateTraj(state0, state1, self.swing_traj_planner))

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
        :return: foot trajectory list (6xpoint_num)
        """
        # For TrajViz
        foot_traj_list = [[] for _ in range(6)]
        index = 0
        while (point_num > 0 and self.state_trajs.is_valid(index)):
            state_traj = self.state_trajs.at(index)
            while (t < 1 and point_num > 0):
                foot_pos_list = state_traj.eval_foot_traj(t, False)
                for j in range(6):
                    foot_traj_list[j].append(foot_pos_list[j])
                point_num -= 1
                t += delta
            index += 1
            t = 0
        return foot_traj_list
