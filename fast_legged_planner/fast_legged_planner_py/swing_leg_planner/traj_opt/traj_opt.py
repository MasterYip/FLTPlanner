'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-13 10:01:31
LastEditors: RaymonYip-NUC11
LastEditTime: 2024-01-15 19:47:29
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/swing_leg_planner/traj_opt/traj_opt.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8

from typing_extensions import deprecated, override
import numpy as np
from abc import abstractmethod, ABCMeta
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b

from ..traj_gen.traj_gen import HermiteSpline, UniBSpline, TimedLinearSpline
from ..cost.cost import CostCollection
from ...perception_interface.gridmap_interface_ros import GridMap_Interface
from .rrt.rrt_interface import Gridmap_SearchSpace, HITSpiderCfg_SearchSpace, OMPL_GridmapSearchSpace, OMPL_HITSpiderCfg_SearchSpace
from ...third_party.rrt_algorithms.src.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic

# OMPL
from ompl import base
from ompl import geometric
################################################
# Initial Trajectory Generation
################################################


def default_1stage_hermite(start, end, v_lift=0.8):
    """Generate a 1-stage Hermite Spline
    :param start: start point
    :param end: end point
    :param v_lift: lift velocity
    """
    v = np.array([0, 0, v_lift])
    return HermiteSpline(
        np.array([start, v, end, -v]))


def default_2stage_hermite(start, end, v_lift=0.8, h_lift=0.3):
    """Generate a 2-stage Hermite Spline
    :param start: start point
    :param end: end point
    :param v_lift: lift velocity
    """
    v = np.array([0, 0, v_lift])
    p_mid = (start+end)*0.5
    p_mid[-1] += h_lift
    v_mid = (start-end)*0.5  # Cardinal Spline
    return HermiteSpline(
        np.array([start, v, p_mid, v_mid, end, -v]))

# TODO: need to add a default spline lib for different robot


def default_bspline(start, end, res=5, k=3, v_lift=0.8, h_lift=0.3):
    """Generate a B-Spline
    :param start: start point
    :param end: end point
    :param res: resolution
    :param k: order
    :param v_lift: lift velocity
    :param h_lift: lift height
    """
    hermite = default_2stage_hermite(start, end, v_lift, h_lift)
    return UniBSpline(np.array([hermite.evaluate(t, normalized=True) for t in np.linspace(0, 1, res)]), k)


################################################
# Optimization Problem Base
################################################


class OptProbBase(object):

    spline = None

    @abstractmethod
    def __init__(self, spline) -> None:
        self.spline = spline
    # @abstractmethod
    # def __init__(self, start, goal) -> None:
    #     pass

    """ Decision """
    @abstractmethod
    def get_decision_var(self):
        """Return the decision variable"""
        pass

    @abstractmethod
    def set_decision_var(self, var):
        """Set the decision variable"""
        pass

    """ Cost """
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

    """ Optimization """
    @abstractmethod
    def init_solution(self):
        pass

    def optimize(self, maxiter=20, use_fprime=False, disp=False):
        return np.array(fmin_bfgs(self.get_cost, self.get_decision_var(),
                                  fprime=self.get_cost_derivative if use_fprime else None,
                                  maxiter=maxiter, full_output=False, disp=disp,
                                  callback=self.set_decision_var)).reshape(-1, self.spline.get_dimen())

    """ Benchmark """
    @abstractmethod
    def in_collision(self):
        pass

    @abstractmethod
    def time_cost(self):
        pass

    @abstractmethod
    def get_grade(self):
        pass

################################################
# Optimization Problem
################################################


""" Hermite Spline """


@deprecated("Too old problem formulation")
class HermiteOptProb(OptProbBase):
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


""" Uniform B-Spline """


class UniBSplineOptProb(OptProbBase):
    def __init__(self, spline: UniBSpline, map_interface: GridMap_Interface):
        super().__init__(spline)
        # self.spline = spline
        self.map_interface = map_interface
        self.sdf_margin = 0.02

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
        return self.kinematic_cost() + self.collision_cost()*20
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
        # End vertical cost
        vec0 = params[1, :]-params[0, :]
        vec1 = params[-1, :]-params[-2, :]
        cost_end = np.linalg.norm(vec0[:2])/np.linalg.norm(vec0) +\
            np.linalg.norm(vec1[:2])/np.linalg.norm(vec1)

        return 2*cost_len + 2*cost_dis + cost_dis2*1 + cost_end*2

    # Collision cost
    def collision_cost(self):
        cost = 0
        # for p in self.get_decision_var().tolist():
        for t in self.get_collsample_index():
            cost += self.point_collision_cost(
                self.spline.evaluate(t, normalized=True))
        return cost

    def get_collsample_index(self):
        # FIXME: How to set the number of sample points
        # Use distance metric temporarily
        end_distance = np.linalg.norm(
            self.spline.get()[0, :]-self.spline.get()[-1, :])
        # FIXME: End point is extracted
        # NOTE: More sample points should be assigned at ends(Use Sin(x) mapping)
        point_num = self.spline.get_n()*int(6*end_distance)
        t_arithmetic = np.linspace(0, 1, point_num)
        # return t_arithmetic
        return 0.5*np.sin(t_arithmetic*np.pi-np.pi/2)+0.5

    def point_collision_cost(self, p):
        c = self.sdf_margin - self.map_interface.sdf_value(p)
        cost = 0
        if c > self.sdf_margin:
            cost = c**2 + c
        elif c > 0:
            cost = c
        return cost

    # FIXME

    def collision_cost_derivative(self):
        derivative = self.get_decision_var()
        for i in range(derivative.shape[0]):
            derivative[i, :] = self.map_interface.sdf_derivative(
                derivative[i, :])
        return derivative.flatten()

    def get_first_collision_index(self, threshold=0.1):
        # FIXME: End point is checked
        for t in self.get_collsample_index():
            if self.point_collision_cost(self.spline.evaluate(t, normalized=True)) > threshold:
                print(self.point_collision_cost(
                    self.spline.evaluate(t, normalized=True)))
                return t
        return None

    def get_max_collision_index(self, threshold=0.1):
        max = 0
        maxt = 0
        for t in self.get_collsample_index():
            value = self.point_collision_cost(
                self.spline.evaluate(t, normalized=True))
            if value > max:
                max = value
                maxt = t
        if max > threshold:
            return maxt
        return None


class Legged_UniBSplineOptProb(UniBSplineOptProb):
    def __init__(self, spline: UniBSpline, torso_traj,
                 map_interface: GridMap_Interface, collmodel):
        super().__init__(spline, map_interface)
        self.collmodel = collmodel
        # FIXME: a function or class?
        self.torso_traj = torso_traj

    @override
    def collision_cost(self):
        cost = 0
        for t in self.get_collsample_index():
            cost += self.collmodel.getCollCost_IK(
                self.spline.evaluate(t, normalized=True),
                self.torso_traj(t))
            # cost += self.point_collision_cost(
            #     self.spline.evaluate(t, normalized=True))
        return cost


""" RRT Search """


class OMPL_RRTBSplineOptProb(object):
    def __init__(self, spline, map_interface,
                 z_margin=.3, obs_clearance=0.04, end_ignore_dia=0.05):
        self.spline = spline
        self.map_interface = map_interface
        self.search_space = OMPL_GridmapSearchSpace(
            map_interface, spline.get_start(), spline.get_end(),
            z_margin=z_margin, obs_clearance=obs_clearance, end_ignore_dia=end_ignore_dia)

        # Create an instance of the problem definition
        self.pdef = base.ProblemDefinition(
            self.search_space.get_space_instance())

        # Set the start and goal states
        start = base.State(self.search_space.get_space_info())
        start[0] = spline.get_start()[0]
        start[1] = spline.get_start()[1]
        start[2] = spline.get_start()[2]
        goal = base.State(self.search_space.get_space_info())
        goal[0] = spline.get_end()[0]
        goal[1] = spline.get_end()[1]
        goal[2] = spline.get_end()[2]
        self.pdef.setStartAndGoalStates(start, goal)

    def optimize(self, maxtime=0.1, type="informed_rrt_star"):
        si = self.search_space.get_space_instance()
        if type == "rrt":
            planner = geometric.RRT(si)
        elif type == "informed_rrt_star":
            planner = geometric.InformedRRTstar(si)
        elif type == "rrt_star":
            planner = geometric.RRTstar(si)

        # Set the problem definition for the planner
        planner.setProblemDefinition(self.pdef)
        # TODO
        # # Set the maximum propagation distance for the planner
        # planner.setRange(0.1)
        # # Set the maximum number of iterations for the planner
        # planner.setMaxIterations(10000)

        # Set the maximum time for the planner
        planner.setup()
        # FIXME: Update the ret strategy
        solved = planner.solve(maxtime)
        if solved:
            path = self.pdef.getSolutionPath()
            self.spline.set(
                np.array([[state[0], state[1], state[2]] for state in path.getStates()]))
        return solved


class OMPL_RRTCfg_OptProb(object):
    def __init__(self, spline, torso_tarj, leg_index,
                 map_interface: GridMap_Interface, robot_interface,
                 end_ignore_dia=0.05):
        self.leg_index = leg_index
        self.torso_tarj = torso_tarj
        self.robot_interface = robot_interface
        self.map_interface = map_interface

        self.spline = spline

        t = self.robot_interface.IKFast_foot(
            leg_index, torso_tarj(0).inverse() * spline.get_start()).tolist()
        t.insert(0, 0.)
        self.start = tuple(t)
        t = self.robot_interface.IKFast_foot(leg_index, torso_tarj(
            1).inverse() * spline.get_end()).tolist()
        t.insert(0, 1.)
        self.end = tuple(t)
        self.search_space = OMPL_HITSpiderCfg_SearchSpace(robot_interface, map_interface,
                                                          torso_tarj, leg_index,
                                                          self.start, self.end,
                                                          end_ignore_dia=end_ignore_dia)
        # OMPL
        # Create an instance of the problem definition
        self.pdef = base.ProblemDefinition(
            self.search_space.get_space_instance())

        # Set the start and goal states
        start = base.State(self.search_space.get_space_info())
        goal = base.State(self.search_space.get_space_info())
        for i in range(len(self.start)):
            start[i] = self.start[i]
            goal[i] = self.end[i]
        self.pdef.setStartAndGoalStates(start, goal)

    def optimize(self, maxtime=2, type="informed_rrt_star"):
        si = self.search_space.get_space_instance()
        if type == "rrt":
            planner = geometric.RRT(si)
        elif type == "informed_rrt_star":
            planner = geometric.InformedRRTstar(si)
        elif type == "rrt_star":
            planner = geometric.RRTstar(si)

        # Set the problem definition for the planner
        planner.setProblemDefinition(self.pdef)
        # TODO
        # # Set the maximum propagation distance for the planner
        # planner.setRange(0.1)
        # # Set the maximum number of iterations for the planner
        # planner.setMaxIterations(10000)

        # Set the maximum time for the planner
        planner.setup()
        solved = planner.solve(maxtime)
        if solved:
            path_cfg = self.pdef.getSolutionPath().getStates()
            workspace_path = []
            for path in path_cfg:
                path = [path[i] for i in range(self.search_space.get_dimen())]
                m = self.robot_interface.get_foot_placement(
                    path[1:], self.leg_index)
                m_world = self.torso_tarj(path[0]) * m
                workspace_path.append(np.concatenate(
                    ([path[0]], m_world.translation)))
            self.spline = TimedLinearSpline(
                np.array(workspace_path))
        else:
            print("OMPL RRT failed")
        return self.spline


class RRTBSplineOptProb(object):
    def __init__(self, spline, map_interface: GridMap_Interface,
                 z_margin=.3, obs_clearance=0.04, end_ignore_dia=0.05):
        self.spline = spline
        self.map_interface = map_interface
        self.search_space = Gridmap_SearchSpace(
            map_interface, spline.get_start(), spline.get_end(),
            z_margin=z_margin, obs_clearance=obs_clearance, end_ignore_dia=end_ignore_dia)

    def optimize(self, Q=np.array([(0.1, 4)]), r=0.01, max_samples=1024, rewire_count=32, prc=0.01):
        self.Q = Q  # length of tree edges
        self.r = r  # length of smallest edge to check for intersection with obstacles
        self.max_samples = max_samples  # max number of samples to take before timing out
        self.rewire_count = rewire_count  # optional, number of nearby branches to rewire
        self.prc = prc  # probability of checking for a connection to goal
        self.rrt = RRTStarBidirectionalHeuristic(
            self.search_space, self.Q, tuple(
                self.spline.get_start()), tuple(self.spline.get_end()),
            self.max_samples, self.r, self.prc, self.rewire_count)
        self.path = self.rrt.rrt_star_bid_h(verbose=False)
        # TODO: update the ret strategy
        if self.path is not None:
            self.spline.set(np.array(self.path))
            return True
        else:
            print("RRT failed")
            return False


class RRTCfg_OptProb(object):
    """RRT Search under HIT Spider Config space"""

    def __init__(self, spline, torso_tarj, leg_index,
                 map_interface: GridMap_Interface, robot_interface,
                 end_ignore_dia=0.05):
        self.leg_index = leg_index
        self.torso_tarj = torso_tarj
        self.robot_interface = robot_interface
        self.map_interface = map_interface

        self.spline = spline

        t = self.robot_interface.IKFast_foot(
            leg_index, torso_tarj(0).inverse() * spline.get_start()).tolist()
        t.insert(0, 0.)
        self.start = tuple(t)
        t = self.robot_interface.IKFast_foot(leg_index, torso_tarj(
            1).inverse() * spline.get_end()).tolist()
        t.insert(0, 1.)
        self.end = tuple(t)
        self.search_space = HITSpiderCfg_SearchSpace(robot_interface, map_interface,
                                                     torso_tarj, leg_index,
                                                     self.start, self.end,
                                                     end_ignore_dia=end_ignore_dia)

    def optimize(self, Q=np.array([(0.1, 4)]), r=0.01, max_samples=1024, rewire_count=32, prc=0.01, verbose=True):
        """
        :param Q: length of tree edges
        :param r: length of smallest edge to check for intersection with obstacles
        :param max_samples: max number of samples to take before timing out
        :param rewire_count: optional, number of nearby branches to rewire
        :param prc: probability of checking for a connection to goal
        :return: `TimedLinearSpline`
        """
        self.Q = Q  # length of tree edges
        self.r = r  # length of smallest edge to check for intersection with obstacles
        self.max_samples = max_samples  # max number of samples to take before timing out
        self.rewire_count = rewire_count  # optional, number of nearby branches to rewire
        self.prc = prc  # probability of checking for a connection to goal
        self.rrt = RRTStarBidirectionalHeuristic(
            self.search_space, self.Q, self.start, self.end,
            self.max_samples, self.r, self.prc, self.rewire_count)
        self.path = self.rrt.rrt_star_bid_h(verbose=verbose)
        if self.path is not None:
            # TODO: How to generate a spline in work space from a timed path in config space?
            workspace_path = []
            for path in self.path:
                m = self.robot_interface.get_foot_placement(
                    path[1:], self.leg_index)
                m_world = self.torso_tarj(path[0]) * m
                workspace_path.append(np.concatenate(
                    ([path[0]], m_world.translation)))
            # print(workspace_path)
            return TimedLinearSpline(np.array(workspace_path))
        else:
            print("RRT failed")
            return self.spline
