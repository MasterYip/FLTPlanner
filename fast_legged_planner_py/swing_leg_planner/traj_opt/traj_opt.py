'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-13 10:01:31
LastEditors: NUC12
LastEditTime: 2023-11-16 14:28:24
FilePath: \Fast-Legged-Planner-Test\fast_legged_planner_py\swing_leg_planner\traj_opt\traj_opt.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8

import numpy as np
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b

from ..traj_gen.traj_gen import HermiteSpline
from ..cost.cost import CostCollection


class TrajOptProblem(object):
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
        self.spline.update(state)
        return self.costs.get_cost(state)

    def get_cost_derivative(self, state):
        self.spline.update(state)
        return self.costs.get_cost_derivative(state)

    def optimize(self, use_fprime=False, maxiter=100, disp=False):
        # return np.array(fmin_bfgs(self.get_cost, self.initial_guess, fprime=self.get_cost_derivative if use_fprime else None,
        #                           maxiter=maxiter, full_output=False)).reshape(-1, 3)
        opt_state = np.array(fmin_bfgs(self.get_cost, self.initial_guess, fprime=self.get_cost_derivative if use_fprime else None,
                                       maxiter=maxiter, full_output=False, disp=disp)).reshape(-1, 3)
        # Fix the start and end point
        # opt_state[0, :] = self.initial_guess[0, :]
        # opt_state[-2, :] = self.initial_guess[-2, :]
        # Fix the start and end point pos&vel
        # opt_state[0:1, :] = self.initial_guess[0:1, :]
        # opt_state[-2:, :] = self.initial_guess[-2:, :]
        return opt_state
