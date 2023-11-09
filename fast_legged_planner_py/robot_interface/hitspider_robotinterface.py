'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-09 21:32:46
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-09 21:52:56
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/hitspider_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-

import numpy as np
import pinocchio as pin
import tempfile


class HITSpider_RobotInterface(object):
    def __init__(self, urdf_str: str = None) -> None:
        self.tmp_urdf = tempfile.NamedTemporaryFile(
            suffix=".urdf", delete=True)
        self.tmp_urdf.write(urdf_str.encode())
        self.robot = pin.RobotWrapper.BuildFromURDF(
            self.tmp_urdf.name, root_joint="link_base")
        self.data = self.robot.createData()

    def get_base_pose(self):
        return self.robot.position(self.data, 0)
