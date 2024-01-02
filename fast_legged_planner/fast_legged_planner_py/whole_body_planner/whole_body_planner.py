'''
Author: NUC12 2205929492@qq.com
Date: 2023-11-17 12:08:34
LastEditors: NUC12
LastEditTime: 2023-11-17 12:08:47
FilePath: \Fast-Legged-Planner-Test\fast_legged_planner_py\whole_body_planner\whole_body_planner.py
Description: file content
'''
#!/usr/bin/env python
# coding=utf-8
import numpy as np
from abc import abstractmethod, ABCMeta
from ..utils.data_structure import CircleQueue
from ..swing_leg_planner.traj_gen.traj_gen import HermiteSpline


# Abstract class
class WholeBodyPlanner(object):
    def __init__(self):
        pass
