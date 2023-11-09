'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 17:56:55
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-09 14:49:15
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/swing_leg_planner/traj_gen/traj_gen.py
Description: file content
'''
# -*- coding: utf-8 -*-
from scipy.interpolate import interp1d
import numpy as np
# from abc import abstractmethod, ABCMeta

# Parameter Mat
BEZIER_MAT = np.array([[1, 0, 0, 0], [-3, 3, 0, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])
HERMITE_MAT = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                        [-3, -2, 3, -1], [2, 1, -2, 1]])
UNI_B_MAT = np.array([[1, 4, 1, 0], [-3, 0, 3, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])/6


def cubic_evaluate(knots, t, para_mat: np.ndarray):
    """
    Evaluate cubic curve at parameter t
    :param knots: control points of cubic curve
    :param t: parameter vector
    :param para_mat: polynomial parameter matrix
    """
    if type(t) is not np.ndarray:
        t = np.array([t])
    t_vec = np.array([np.ones(t.shape), t, t**2, t**3]).T
    point = t_vec @ para_mat @ knots
    return point


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
    :param knots: control points of bezier curve
    :param t: parameter vector
    :return: point on bezier curve
    """
    return cubic_evaluate(knots, t, BEZIER_MAT)
