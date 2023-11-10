'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 15:47:47
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 09:28:52
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/base_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-
from abc import abstractmethod, ABCMeta


class Base_RobotInterface(metaclass=ABCMeta):
    def __init__(self) -> None:
        pass

    # Debug
    def print_joints(self):
        for i in range(self.robot.model.njoints):
            print(i, self.robot.model.names[i])

    def print_frames(self):
        for i in range(self.robot.model.nframes):
            print(i, self.robot.model.frames[i].name)
