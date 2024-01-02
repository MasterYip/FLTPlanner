'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 15:47:47
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-12-29 19:42:17
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/base_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-
import os
import hashlib
import tempfile
import numpy as np
import pinocchio as pin
from abc import ABCMeta
from ..third_party.meshcat_viewer_wrapper.visualizer import colors

Load_Meshcat = False
MeshcatFound = False
if Load_Meshcat:
    try:
        from ..third_party.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
        MeshcatFound = True
    except ImportError:
        pass


class Base_RobotInterface(metaclass=ABCMeta):
    def __init__(self, urdf: str = None, package_dirs=None) -> None:
        """
        :param urdf: URDF file path OR URDF string
        :param package_dirs: list of package directories containing meshes
        """
        super().__init__()
        # Parse URDF
        if urdf.startswith("<"):
            tmp_urdf = tempfile.NamedTemporaryFile(suffix=".urdf", delete=True)
            tmp_urdf.write(urdf.encode())
            self.robot = pin.RobotWrapper.BuildFromURDF(
                tmp_urdf.name, package_dirs)
        elif urdf.endswith(".urdf") and os.path.isfile(urdf):
            self.robot = pin.RobotWrapper.BuildFromURDF(urdf, package_dirs)
        else:
            # print("\033[91mWarning: URDF file or string are not specified.\033[0m")
            raise ValueError("URDF file or string are not specified.")
        # Meshcat Vis
        if MeshcatFound:
            self.viz = MeshcatVisualizer(self.robot)

    def update_kinematics(self, q):
        pin.forwardKinematics(self.robot.model, self.robot.data, q)

    def get_frameid(self, frame_name):
        return self.robot.model.getFrameId(frame_name)

    def get_frame_placement(self, q, frame_name, update_kinematics=True):
        """
        :param q: joint angles
        :param frame_name: frame name
        """
        return self.robot.framePlacement(q, self.get_frameid(frame_name), update_kinematics)

    # Debug
    def print_joints(self):
        for i in range(self.robot.model.njoints):
            print(i, self.robot.model.names[i])

    def print_frames(self):
        for i in range(self.robot.model.nframes):
            print(i, self.robot.model.frames[i].name)

    # Viz
    def viz_add_sphere(self, pos, radius=0.1, color=colors.red, name=None):
        if name is None:
            name = hashlib.md5(str(np.random.rand()).encode()).hexdigest()
        self.viz.addSphere("world/Sphere/"+name, radius, color)
        self.viz.applyConfiguration(
            "world/Sphere/" + name, pin.SE3(np.eye(3), np.array(pos)))

    def viz_clear(self, name_space="world/Sphere"):
        self.viz.delete(name_space)
