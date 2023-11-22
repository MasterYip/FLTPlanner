'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-13 10:01:31
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-22 17:34:52
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_opt//traj_opt.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8

from typing_extensions import override
import numpy as np
from abc import abstractmethod, ABCMeta
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b

from ..traj_gen.traj_gen import HermiteSpline, UniBSpline
from ..cost.cost import CostCollection
from ...perception_interface.gridmap_interface_ros import GridMap_Interface


class SplineOptBase(object):

    spline = None

    def __init__(self, spline) -> None:
        self.spline = spline

    @abstractmethod
    def get_decision_var(self):
        """Return the decision variable"""
        pass

    @abstractmethod
    def set_decision_var(self, var):
        """Set the decision variable"""
        pass

    @abstractmethod
    def get_cost(self, state):
        """Update the spline and return the cost"""
        pass

    @abstractmethod
    def get_cost_derivative(self, state):
        """Update the spline and return the cost derivative"""
        pass

    @abstractmethod
    def get_quadratic_approx(self, state):
        pass

    def optimize(self, maxiter=100, use_fprime=False, disp=False):
        return np.array(fmin_bfgs(self.get_cost, self.get_decision_var(),
                                  fprime=self.get_cost_derivative if use_fprime else None,
                                  maxiter=maxiter, full_output=False, disp=disp,
                                  callback=self.set_decision_var)).reshape(-1, self.spline.get_dimen())


class HermiteOptProb(SplineOptBase):
    def __init__(self, spline: HermiteSpline, costs: CostCollection, constraints, initial_guess, method='L-BFGS-B'):
        self.costs = costs
        self.constraints = constraints
        self.initial_guess = initial_guess
        self.method = method
        self.spline = spline

    def get_cost(self, state):
        # update knots, literally np.array([p0, v0,..., pn, vn])
        # Fix the start and end point
        state = state.reshape(-1, 3)
        state[0, :] = self.initial_guess[0, :]
        state[-2, :] = self.initial_guess[-2, :]
        # Fix the start and end point pos&vel
        # state[0:1, :] = self.initial_guess[0:1, :]
        # state[-2:, :] = self.initial_guess[-2:, :]
        self.spline.set(state)
        return self.costs.get_cost(state)

    def get_cost_derivative(self, state):
        self.spline.set(state)
        return self.costs.get_cost_derivative(state)

    def optimize(self, use_fprime=False, maxiter=100, disp=False):
        # return np.array(fmin_bfgs(self.get_cost, self.initial_guess, fprime=self.get_cost_derivative if use_fprime else None,
        #                           maxiter=maxiter, full_output=False)).reshape(-1, 3)
        opt_state = np.array(fmin_bfgs(self.get_cost, self.initial_guess, fprime=self.get_cost_derivative if use_fprime else None,
                                       maxiter=maxiter, full_output=False, disp=disp)).reshape(-1, 3)
        # NOTE: This enable consecutive optimization
        self.initial_guess = opt_state
        # Fix the start and end point
        # opt_state[0, :] = self.initial_guess[0, :]
        # opt_state[-2, :] = self.initial_guess[-2, :]
        # Fix the start and end point pos&vel
        # opt_state[0:1, :] = self.initial_guess[0:1, :]
        # opt_state[-2:, :] = self.initial_guess[-2:, :]
        return opt_state


class UniBSplineOptProb(SplineOptBase):
    def __init__(self, spline: UniBSpline, map_interface: GridMap_Interface):
        super().__init__(spline)
        self.map_interface = map_interface

    @override
    def get_decision_var(self):
        return self.spline.get()[1:-1, :]

    @override
    def set_decision_var(self, state):
        state = np.vstack(
            [self.spline.get()[0, :], state.reshape(-1, self.spline.get_dimen()), self.spline.get()[-1, :]])
        self.spline.set(state)

    @override
    def get_cost(self, state):
        self.set_decision_var(state)
        return self.kinematic_cost() + self.collision_cost()*50
        # return self.collision_cost()*10

    @override
    def get_cost_derivative(self, state):
        # FIXME: This is not correct
        # self.set_decision_var(state)
        # return self.kinematic_cost_derivative() + self.collision_cost_derivative()*10
        return self.collision_cost_derivative()*10

    # Kinematic cost
    def kinematic_cost(self):
        params = self.spline.get()
        # Length cost
        cost_len = 0
        for i in range(self.spline.get_n()-1):
            cost_len += np.linalg.norm(params[i]-params[i+1])
        # Distribution cost
        cost_dis = 0
        cost_dis2 = 0
        for i in range(self.spline.get_n()-2):
            cost_dis += np.linalg.norm(params[i]+params[i+2]-2*params[i+1])
            cost_dis2 += abs(np.linalg.norm(params[i]-params[i+1]) -
                             np.linalg.norm(params[i+1]-params[i+2]))

        return 0.2*cost_len + 0.5*cost_dis + cost_dis2*5

    # Collision cost
    def collision_cost(self):
        sdf_margin = 0.1

        def point_collision_cost(p):
            c = sdf_margin - self.map_interface.sdf_value(p)
            cost = 0
            if c > sdf_margin:
                cost = c**2 + c
            elif c > 0:
                cost = c
            return cost
        cost = 0
        # for p in self.get_decision_var().tolist():
        for p in [self.spline.evaluate(t, normalized=True) for t in np.linspace(0, 1, self.spline.get_n())]:
            cost += point_collision_cost(p)
        return cost

    # FIXME

    def collision_cost_derivative(self):
        derivative = self.get_decision_var()
        for i in range(derivative.shape[0]):
            derivative[i, :] = self.map_interface.sdf_derivative(
                derivative[i, :])
        return derivative.flatten()
