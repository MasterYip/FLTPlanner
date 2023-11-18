'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-16 10:25:09
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-16 17:00:53
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/swing_leg_planner/cost/cost.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from abc import abstractmethod, ABCMeta
from ..traj_gen.traj_gen import SplineBase, HermiteSpline
try:
    from ...perception_interface.gridmap_interface_ros import GridMap_Interface
except ImportError:
    print("Import Error: GridMap_Interface not found!")


class StateCost:
    # TODO: derived from ocs2, temporarily not used
    def __init__(self):
        pass

    def get_value(self, time, state, target_trajectories, pre_comp):
        raise NotImplementedError(
            "Subclasses must implement the get_value method")

    def get_quadratic_approximation(self, time, state, target_trajectories, pre_comp):
        raise NotImplementedError(
            "Subclasses must implement the get_quadratic_approximation method")


class CostBase:
    def __init__(self):
        pass

    @abstractmethod
    def get_cost(self, state):
        raise NotImplementedError(
            "Subclasses must implement the get_cost method")

    @abstractmethod
    def get_cost_derivative(self, state):
        raise NotImplementedError(
            "Subclasses must implement the get_cost_derivative method")

    def get_cost_hessian(self, state):
        raise NotImplementedError(
            "Subclasses must implement the get_cost_hessian method")

    def get_cost_quadratic_approximation(self, state):
        raise NotImplementedError(
            "Subclasses must implement the get_cost_quadratic_approximation method")


class CostCollection(object):
    def __init__(self, cost_list: list):
        self.cost_list = cost_list

    def get_cost(self, state):
        cost = 0
        for cost_i in self.cost_list:
            cost += cost_i.get_cost(state)
        return cost

    def get_cost_derivative(self, state):
        derivative = np.zeros(state.shape)
        for cost_i in self.cost_list:
            derivative += cost_i.get_cost_derivative(state)
        return derivative


class KinematicCost(CostBase):
    def __init__(self, spline: HermiteSpline):
        super().__init__()
        self.spline = spline

    def get_length_cost(self):
        cost = 0
        pos_list = self.spline.get_poslist()
        for i in range(pos_list.shape[0]-1):
            cost += np.linalg.norm(pos_list[i]-pos_list[i+1])
        return cost

    def get_vel_cost(self):
        cost = 0
        vel_list = self.spline.get_vellist()
        for i in range(vel_list.shape[0]-1):
            cost += np.linalg.norm(vel_list[i]-vel_list[i+1])
        return cost

    def get_cost(self, state):
        return self.get_length_cost() + self.get_vel_cost()

    # def get_cost_derivative(self, state):
    #     derivative = []
    #     for p in self.spline.get_poslist():
    #         derivative.append(p / np.linalg.norm(p))
    #     return np.array(derivative)


class CollisionCost(CostBase):

    def __init__(self, spline: HermiteSpline, interface):
        # def __init__(self, spline: HermiteSpline, interface: GridMap_Interface):
        """
        :param spline: spline to be evaluated (reference)
        :param interface: gridmap interface GridMap_Interface (reference)
        """
        super().__init__()
        self.spline = spline
        self.interface = interface
        self.sdf_margin = 0.2

        self.resolution = 10
        self.ts = np.linspace(
            self.spline.t_range[0], self.spline.t_range[1], self.resolution)
        pass

    def get_cost(self, state=None):
        cost = 0
        # for p in self.spline.get_poslist():
        # TODO: waiting for Spline.evaluate optimization
        for p in [self.spline.evaluate(t) for t in self.ts]:
            c = self.sdf_margin - self.interface.sdf_value(p)
            if c > 0:
                cost += c
        return cost

    # FIXME:
    def get_cost_derivative(self):
        derivative = []
        for p in self.spline.get_poslist():
            # Derivative by position
            if self.interface.sdf_value(p) < self.sdf_margin:
                # Direction: opposite to gradient
                derivative.append(-self.interface.sdf_derivative(p))
            else:
                derivative.append(np.zeros(3))
            # Derivative by velocity
            derivative.append(np.zeros(3))
        return np.array(derivative)
