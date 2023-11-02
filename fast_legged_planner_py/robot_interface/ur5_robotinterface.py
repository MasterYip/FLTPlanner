'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 17:18:15
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-02 17:59:21
FilePath: /Fast-Legged-Planner-Test/fast_legged_planner_py/robot_interface/ur5_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-
from .base_robotinterface import Base_RobotInterface
from ..third_party.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer, colors
import example_robot_data as robex
from numpy.linalg import norm
from scipy.optimize import fmin_bfgs
import pinocchio as pin


class UR5_RobotInterface(Base_RobotInterface):

    def __init__(self):
        self.robot = robex.load('ur5')
        self.viz = MeshcatVisualizer(self.robot)

    def get_target_cfg_3D(self, target, last_q=None):
        """Get the target configuration

        Args:
            target (np.ndarray): target 3d position
            last_q (_type_, optional): last config, to speed up solver.
        """
        def cost(q):
            '''Compute score from a configuration'''
            m = self.robot.framePlacement(q, 22)
            p = m.translation
            # offset = m.rotation[:, 2] * radius
            # return norm(p +  offset - target)**2
            return norm(p - target)**2

        if last_q is None:
            last_q = self.robot.q0
        return fmin_bfgs(cost, last_q, disp=False, gtol=1e-4)

    def vis_target_reach_3D(self, target, last_q=None):
        ball_position = list(target)
        ball_rad = 0.07
        ball_id = 'world/ball'
        self.viz.addSphere(ball_id, ball_rad, colors.red)
        q_ball = ball_position + [1, 0, 0, 0]
        self.viz.applyConfiguration(ball_id, q_ball)
        cfg = self.get_target_cfg_3D(target, last_q)
        self.viz.display(cfg)
        return cfg

    def get_target_cfg_6D(self, SE3target, last_q=None):
        """_summary_

        Args:
            SE3target (_type_): _description_
            last_q (_type_, optional): _description_. Defaults to None.
        """
        def cost(q):
            '''Compute score from a configuration'''
            M = self.robot.framePlacement(q, 22)
            return norm(pin.log(SE3target.inverse()*M).vector)

        if last_q is None:
            last_q = self.robot.q0
        return fmin_bfgs(cost, last_q, disp=False, gtol=1e-4)

    def vis_target_reach_6D(self, SE3target, last_q=None):
        cube_dimension = [0.02, 0.04, 0.06]
        cube_id = 'world/cube'
        self.viz.addBox(cube_id, cube_dimension, colors.red)
        q_ball = pin.SE3ToXYZQUATtuple(SE3target)
        self.viz.applyConfiguration(cube_id, q_ball)
        cfg = self.get_target_cfg_6D(SE3target, last_q)
        self.viz.display(cfg)
        return cfg
