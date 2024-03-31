'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-09 21:32:46
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-12-13 15:14:45
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//robot_interface//hitspider_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-

import numpy as np
import pinocchio as pin
from typing_extensions import deprecated
from .base_robotinterface import Base_RobotInterface
# from .pin_IK import pinIK
from scipy.optimize import fmin_bfgs
from numpy.linalg import norm
from ..third_party.meshcat_viewer_wrapper.visualizer import colors
from .pyikfast import pyikfast_hitspider as ik

JOINT_STATE_NAME = ["joint_lf_1", "joint_lf_2", "joint_lf_3",
                    "joint_lh_1", "joint_lh_2", "joint_lh_3",
                    "joint_lm_1", "joint_lm_2", "joint_lm_3",
                    "joint_rf_1", "joint_rf_2", "joint_rf_3",
                    "joint_rh_1", "joint_rh_2", "joint_rh_3",
                    "joint_rm_1", "joint_rm_2", "joint_rm_3"]

FOOT_LINK_NAME = ["link_lf_foot", "link_lh_foot", "link_lm_foot",
                  "link_rf_foot", "link_rh_foot", "link_rm_foot"]

LEG_CFG_DEG_RANGE = [100, 120, 120]  # Deg
LEG_CFG_SPACE = np.array([[np.deg2rad(-LEG_CFG_DEG_RANGE[0]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[0])],
                          [np.deg2rad(-LEG_CFG_DEG_RANGE[1]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[1])],
                          [np.deg2rad(-LEG_CFG_DEG_RANGE[2]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[2])]])


class HITSpider_RobotInterface(Base_RobotInterface):
    def __init__(self, urdf: str, package_dirs=None) -> None:
        super().__init__(urdf, package_dirs)
        # self.collmodel = HITSpider_Collision_Model()
        self.rot_mat = [0, np.pi/3*2, np.pi/3, -np.pi/3, -np.pi, -np.pi/3*2]

    def get_leg_cfg_space(self):
        return LEG_CFG_SPACE

    def get_full_q(self, q_leg, foot_index):
        q = np.zeros(18)
        index = foot_index*3
        q[index:index+3] = q_leg
        return q

    def get_foot_placement(self, q_leg, foot_index):
        return self.get_frame_placement(self.get_full_q(q_leg, foot_index), FOOT_LINK_NAME[foot_index])

    def check_legcfg_valid(self, q_leg):
        for i in range(3):
            if q_leg[i] < LEG_CFG_SPACE[i][0] or q_leg[i] > LEG_CFG_SPACE[i][1]:
                return False
        return True

    def IKFast_foot(self, foot_index, target: np.ndarray, valid_check=True, fall_back=True, ray_approx=True):
        """Get the target configuration using IKFast
        Args:
            foot_index (int): 0-5
            target (np.ndarray(3,)): target 3d position relative to base
            valid_check (bool, optional): check if the solution is valid. Defaults to True.
            fall_back (bool, optional): fall back to IK_foot if IKFast failed. Defaults to True.
            ray_approx (bool, optional): use ray approximation. Defaults to True.
        """
        th_2 = self.rot_mat[foot_index]/2
        m = pin.XYZQUATToSE3([0, 0, 0, 0, 0, np.sin(th_2), np.cos(th_2)])
        pos = m.actInv(target)
        # Ray approximation
        ray_approx_factor = 0.9
        max_try = 10 if ray_approx else 1
        while max_try > 0:
            sol = ik.IKFast_trans3D(list(pos))
            for i in range(len(sol)):
                if self.check_legcfg_valid(sol[i]) or not valid_check:  # Valid check
                    return np.array(sol[i])
            max_try -= 1
            pos *= ray_approx_factor
        # Fall back
        if fall_back is True:
            print("Warning: IKFast failed, fall back on IK_foot")
            return self.IK_foot(foot_index, target)  # Fall back
        else:
            raise Exception("IKFast failed")

    def IK_foot(self, foot_index, target, q_leg0=np.zeros(3)):
        """Get the target configuration
        Args:
            target (np.ndarray(3,)): target 3d position relative to base
            q0 (np.ndarray(3,), optional): last config, to speed up solver.
        """
        def cost(q_leg):
            '''Compute score from a configuration'''
            m = self.get_frame_placement(self.get_full_q(
                q_leg, foot_index), FOOT_LINK_NAME[foot_index])
            p = m.translation
            # offset = m.rotation[:, 2] * radius
            # return norm(p +  offset - target)**2
            return norm(p - target)**2

        return fmin_bfgs(cost, q_leg0, disp=False, gtol=1e-4)

    def IKFast_foots(self, target_list, valid_check=True, fall_back=True, ray_approx=True):
        q = np.zeros(18)
        for i in range(6):
            q[3*i:3*i+3] = self.IKFast_foot(i, target_list[i],
                                            valid_check, fall_back, ray_approx)
        return q

    def IK_foots(self, target_list, q0=np.zeros(18)):
        q = q0
        for i in range(6):
            q[3*i:3*i+3] = self.IK_foot(i, target_list[i])
        return q

    # Collision Viz (Meshcat)
    @deprecated("Do not us this function")
    def vis_collision_model(self, q):
        for collsphere in self.collmodel.collspheres:
            viz_id = "world/collsphere/"+collsphere.frame_name
            self.viz.addSphere(
                viz_id, collsphere.radius, colors.green_transparent)
            self.viz.applyConfiguration(
                viz_id, self.get_frame_placement(q, collsphere.frame_name))


