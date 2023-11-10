'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-02 17:18:15
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 17:40:38
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/ur5_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-
import numpy as np
from .base_robotinterface import Base_RobotInterface
from .pin_IK import pinIK
from ..third_party.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer, colors
import example_robot_data as robex
from numpy.linalg import norm
from scipy.optimize import fmin_bfgs
import pinocchio as pin

from ..swing_leg_planner.collision_check.collision_check import UR5_Collision_Model


class UR5_RobotInterface(Base_RobotInterface):

    def __init__(self):
        # super().__init__()
        self.robot = robex.load('ur5')
        self.collmodel = UR5_Collision_Model()
        self.viz = MeshcatVisualizer(self.robot)
        self.settings = {
            'collmodel_viz': True
        }

    def pinIK_end3D(self, target, last_q=None):
        if last_q is None:
            last_q = self.robot.q0
        q, error, success = pinIK(
            self.robot.model, self.robot.data, pin.SE3(np.eye(3), target), last_q, 22, mode=3)
        return q

    def IK_end3D(self, target, last_q=None):
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

    def vis_target_reach_3D(self, target, last_q=None, use_pinIK=False):
        ball_position = list(target)
        ball_rad = 0.04
        ball_id = 'world/ball'
        self.viz.addSphere(ball_id, ball_rad, colors.red_transparent)
        q_ball = ball_position + [1, 0, 0, 0]
        self.viz.applyConfiguration(ball_id, q_ball)
        if use_pinIK:
            cfg = self.pinIK_end3D(target, last_q)
        else:
            cfg = self.IK_end3D(target, last_q)
        self.viz.display(cfg)
        if self.settings['collmodel_viz']:
            self.vis_collision_model(cfg)
        return cfg

    def pinIK_end6D(self, SE3target, last_q=None):
        if last_q is None:
            last_q = self.robot.q0
        q, error, success = pinIK(
            self.robot.model, self.robot.data, SE3target, last_q, 22, mode=6)
        return q

    def IK_end6D(self, SE3target, last_q=None):
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

    def vis_target_reach_6D(self, SE3target, last_q=None, use_pinIK=False):
        cube_dimension = [0.02, 0.04, 0.06]
        cube_id = 'world/cube'
        self.viz.addBox(cube_id, cube_dimension, colors.red_transparent)
        q_ball = pin.SE3ToXYZQUATtuple(SE3target)
        self.viz.applyConfiguration(cube_id, q_ball)
        if use_pinIK:
            cfg = self.pinIK_end6D(SE3target, last_q)
        else:
            cfg = self.IK_end6D(SE3target, last_q)
        self.viz.display(cfg)
        if self.settings['collmodel_viz']:
            self.vis_collision_model(cfg)
        return cfg

    # Collision Check
    def vis_collision_model(self, q):
        for collsphere in self.collmodel.collspheres:
            viz_id = "world/collsphere/"+collsphere.frame_name
            self.viz.addSphere(
                viz_id, collsphere.radius, colors.green_transparent)
            self.viz.applyConfiguration(
                viz_id, self.robot.framePlacement(q, self.robot.model.getFrameId(collsphere.frame_name)))

