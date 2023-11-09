'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 17:56:55
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-09 17:00:38
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/swing_leg_planner/traj_gen/traj_gen.py
Description: file content
'''
# -*- coding: utf-8 -*-
# from scipy.interpolate import interp1d, LinearNDInterpolator
import numpy as np
# from abc import abstractmethod, ABCMeta

# Parameter Mat
LINEAR_MAT = np.array([[1, 0], [-1, 1]])

BEZIER_MAT = np.array([[1, 0, 0, 0], [-3, 3, 0, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])
HERMITE_MAT = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                        [-3, -2, 3, -1], [2, 1, -2, 1]])
UNI_B_MAT = np.array([[1, 4, 1, 0], [-3, 0, 3, 0],
                      [3, -6, 3, 0], [-1, 3, -3, 1]])/6


def linear_evaluate(knots: np.ndarray, t):
    """
    Evaluate linear curve at parameter t
    :param knots: control points of linear curve with shape (2 row, x col)
    """
    if type(t) is not np.ndarray:
        t = np.array([t])
    t_vec = np.array([np.ones(t.shape), t]).T
    return t_vec @ LINEAR_MAT @ knots


def cubic_evaluate(knots: np.ndarray, t, para_mat: np.ndarray):
    """
    Evaluate cubic curve at parameter t
    :param knots: control points of cubic curve with shape (4 row, x col)
    :param t: parameter vector range from 0 to 1
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
