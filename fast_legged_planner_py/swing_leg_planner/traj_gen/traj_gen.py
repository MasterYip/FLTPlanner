'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 17:56:55
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-21 17:48:16
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//traj_gen//traj_gen.py
Description: file content
'''
# -*- coding: utf-8 -*-
from typing_extensions import deprecated, override
from scipy.interpolate import BSpline, CubicSpline
import numpy as np
from scipy import interpolate
from abc import abstractmethod, ABCMeta

# Parameter Mat
LINEAR_MAT = np.array([[1, 0], [-1, 1]])

BEZIER_MAT = np.array([[1, 0, 0, 0], [-3, 3, 0, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])
HERMITE_MAT = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                        [-3, -2, 3, -1], [2, 1, -2, 1]])
UNI_B_MAT = np.array([[1, 4, 1, 0], [-3, 0, 3, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])/6

################################################
# Linear Curve
################################################


def linear_evaluate(knots: np.ndarray, t):
    """
    Evaluate linear curve at parameter t
    :param knots: control points of linear curve with shape (2 row, x col)
    """
    if type(t) is not np.ndarray:
        t = np.array([t])
    t_vec = np.array([np.ones(t.shape), t]).T
    return (t_vec @ LINEAR_MAT @ knots).flatten()

################################################
# Cubic Curve
################################################


def cubic_evaluate(knots: np.ndarray, t, para_mat: np.ndarray, eval_mode="pos"):
    """
    Evaluate cubic curve at parameter t
    :param knots: control points of cubic curve with shape (4 row, x col)
    :param t: parameter vector range from 0 to 1
    :param para_mat: polynomial parameter matrix
    :param eval_mode: "pos" or "vel"
    """
    if type(t) is not np.ndarray:
        t = np.array([t])
    if eval_mode == "pos":
        t_vec = np.array([np.ones(t.shape), t, t**2, t**3]).T
    elif eval_mode == "vel":
        t_vec = np.array([np.zeros(t.shape), np.ones(t.shape), 2*t, 3*t**2]).T
    else:
        raise ValueError("Invalid eval_mode")
    point = t_vec @ para_mat @ knots
    if point.shape[0] == 1:
        return point.flatten()
    else:
        return point


@deprecated("Not used")
def bezier_evaluate(knots, t):
    """
    Evaluate bezier curve at parameter t
    :param knots: control points of bezier curve (ndarray)
    :param t: parameter
    :return: point on bezier curve
    """
    n = len(knots)
    point = np.zeros((1, 2))
    for i in range(n):
        point += knots[i] * np.math.factorial(n - 1) / (
            np.math.factorial(i) * np.math.factorial(n - 1 - i)) * np.power(t, i) * np.power(1 - t, n - 1 - i)
    return point


def cubic_bezier_evaluate(knots: np.ndarray, t):
    """
    Evaluate cubic bezier curve at parameter t
    :param knots: control points, literally `np.ndarray([p1, p2, p3, p4])`
    :param t: parameter vector range from 0 to 1
    :return: point on bezier curve
    """
    return cubic_evaluate(knots, t, BEZIER_MAT)


def cubic_hermite_evaluate(knots: np.ndarray, t):
    """
    Evaluate cubic hermite curve at parameter t
    :param knots: control points, literally `np.ndarray([p0, v0, p1, v1])`
    :param t: parameter vector range from 0 to 1
    :return: point on hermite curve
    """
    return cubic_evaluate(knots, t, HERMITE_MAT)

################################################
# Cubic Spline
################################################


class SplineBase(object):

    params = None

    @abstractmethod
    def evaluate(self, t: float, d_order: int = 0, normalized: bool = False):
        raise NotImplementedError(
            "Subclasses must implement the insert method")

    @abstractmethod
    def set(self, params: np.ndarray):
        raise NotImplementedError(
            "Subclasses must implement the insert method")

    @abstractmethod
    def get(self):
        raise NotImplementedError(
            "Subclasses must implement the insert method")

    def insert(self, t):
        raise NotImplementedError(
            "Subclasses must implement the insert method")


class HermiteSpline(SplineBase):
    """
    Hermite spline class
    """

    def __init__(self, params: np.ndarray):
        """Hermit spline initialization
        :param params: control points, literally `np.ndarray([p0, v0, p1, v1,..., pn, vn])`
        """
        self.set(params)
        self.para_mat = HERMITE_MAT

    @override
    def set(self, params: np.ndarray):
        self.params = params
        self.n = params.shape[0]  # Hermite case: pos poitns = n/2
        self.t_range = [0, self.n//2-1]

    @override
    def get(self):
        return self.params

    @override
    def evaluate(self, t, eval_mode="pos"):
        # TODO: implement t_vec input
        if t < self.t_range[0] or t > self.t_range[1]:
            raise ValueError("Parameter t out of range")
        index = 2*int(t)
        if index == self.n-2:  # Special case
            return cubic_evaluate(self.params[-4:], 1, self.para_mat, eval_mode)
        return cubic_evaluate(self.params[index:index+4], t-int(t), self.para_mat, eval_mode)

    def evaluate_normalized(self, t_norm):
        """Evaluate spline at normalized parameter t
        :param t_norm: normalized parameter t
        """
        return self.evaluate(t_norm*(self.t_range[1]-self.t_range[0])+self.t_range[0])

    @override
    @deprecated("Insert will change the spline")
    def insert(self, t_list: list):
        """Insert knot at normalized parameter t
        :param t_norm_list: list of normalized parameter t
        """
        knots = self.params
        t_list.sort(reverse=True)
        para_inv = np.linalg.inv(self.para_mat)
        for t in t_list:
            tf = t-int(t)  # t fraction
            P = knots[2*int(t):2*int(t)+4]
            t_diag1 = np.array([[1, 0, 0, 0], [0, tf, 0, 0],
                                [0, 0, tf**2, 0], [0, 0, 0, tf**3]])
            P_new1 = (para_inv @ t_diag1 @ self.para_mat @ P)
            t_diag2 = np.array([[1, tf, tf**2, tf**3],
                                [0, 1-tf, 2*tf*(1-tf), 3*tf**2*(1-tf)],
                                [0, 0, 2*(1-tf)**2, 3*tf*(1-tf)**2],
                                [0, 0, 0, (1-tf)**3]])
            P_new2 = (para_inv @ t_diag2 @ self.para_mat @ P)
            knots[2*int(t)+1] = P_new1[1]  # Vel change at current knot
            knots[2*int(t)+3] = P_new2[3]  # Vel change at next knot
            knots = np.insert(knots, 2*int(t)+2,
                              P_new1[2:], axis=0)  # Knot insert

        self.set(knots)

    @deprecated("Insert will change the spline")
    def insert_normalized(self, t_norm_list: list):
        """Insert knot at normalized parameter t
        :param t_norm_list: list of normalized parameter t
        """
        t_list = [t_norm*(self.t_range[1]-self.t_range[0]) +
                  self.t_range[0] for t_norm in t_norm_list]
        self.insert(t_list)

    @deprecated("Please use set/get methods instead")
    def update(self, params):
        self.params = params
        self.n = params.shape[0]  # Hermite case: pos poitns = n/2
        self.t_range = [0, self.n//2-1]

    def get_range(self):
        return self.t_range

    def get_poslist(self):
        return self.params[::2]

    def get_vellist(self):
        return self.params[1::2]


class UniBSpline(SplineBase):
    def __init__(self, params: np.ndarray, k=3):
        """Uniform B-Spline initialization
        TODO: Note: Start at p0 end at pn
        :param params: control points, literally `np.ndarray([p0, p1, p2, p3,..., pn])`
        """
        self._k = k
        self.set(params)
        # self.para_mat = UNI_B_MAT

    @override
    def set(self, params: np.ndarray):
        self.params = params
        self.n = params.shape[0]
        self.t_range = [0, self.n-1]
        self._t = np.concatenate((np.zeros(self._k), np.arange(
            self.n), np.ones(self._k)*self.t_range[1]))
        dimen = params.shape[1]
        self.bspline = interpolate.BSpline(self._t, np.concatenate(
            (self.params[0].reshape(1, dimen), self.params, self.params[-1].reshape(1, dimen))), self._k)

    @override
    def get(self):
        return self.params

    @override
    def evaluate(self, t, d_order: int = 0, normalized: bool = False):
        return self.bspline(t)
