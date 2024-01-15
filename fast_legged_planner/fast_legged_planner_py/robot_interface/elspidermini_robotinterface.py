'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-12-13 13:57:14
LastEditors: RaymonYip-NUC11
LastEditTime: 2024-01-15 11:47:50
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/elspidermini_robotinterface.py
Description: file content
'''

# -*- coding: utf-8 -*-

import numpy as np
import pinocchio as pin
from .base_robotinterface import Base_RobotInterface
from scipy.optimize import fmin_bfgs
from numpy.linalg import norm
from .pyikfast import pyikfast_el_mini as ik
from .pyikfast import pyikfast_el_mini_back as ik_back

JOINT_STATE_NAME = ["RF_HAA", "RF_HFE", "RF_KFE",
                    "RM_HAA", "RM_HFE", "RM_KFE",
                    "RB_HAA", "RB_HFE", "RB_KFE",
                    "LF_HAA", "LF_HFE", "LF_KFE",
                    "LM_HAA", "LM_HFE", "LM_KFE",
                    "LB_HAA", "LB_HFE", "LB_KFE"]

FOOT_LINK_NAME = ["RF_FOOT", "RM_FOOT", "RB_FOOT",
                  "LF_FOOT", "LM_FOOT", "LB_FOOT"]

LEG_CFG_DEG_RANGE = [100, 120, 120]  # Deg
LEG_CFG_SPACE = np.array([[np.deg2rad(-LEG_CFG_DEG_RANGE[0]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[0])],
                          [np.deg2rad(-LEG_CFG_DEG_RANGE[1]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[1])],
                          [np.deg2rad(-LEG_CFG_DEG_RANGE[2]),
                           np.deg2rad(LEG_CFG_DEG_RANGE[2])]])


class ElSpiderMini_RobotInterface(Base_RobotInterface):
    pin_leg_remap = [4, 5, 3, 1, 2, 0]

    def __init__(self, urdf: str, package_dirs=None) -> None:
        super().__init__(urdf, package_dirs)
        # self.collmodel = HITSpider_Collision_Model()
        # mirror-axis(x,y,z), after-mirror-translation(x,y,z)
        self.mirror_trans = [(np.array((1, 1, 1)), np.array((0, 0, 0))),
                             (np.array((1, 1, 1)), np.array((0.3, 0.06, 0))),
                             (np.array((1, -1, 1)), np.array((0, 0, 0))),
                             (np.array((1, -1, 1)), np.array((0, 0, 0))),
                             (np.array((1, -1, 1)), np.array((0.3, 0.06, 0))),
                             (np.array((1, 1, 1)), np.array((0, 0, 0))),]

    def get_leg_cfg_space(self):
        return LEG_CFG_SPACE

    def get_full_q(self, q_leg, foot_index):
        q = np.zeros(18)
        index = self.pin_leg_remap[foot_index]*3
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
        pos = target*self.mirror_trans[foot_index][0] + \
            self.mirror_trans[foot_index][1]
        # Ray approximation
        ray_approx_factor = 0.9
        max_try = 10 if ray_approx else 1
        while max_try > 0:
            if foot_index in [2, 5]:
                sol = ik_back.IKFast_trans3D(list(pos))
            else:
                sol = ik.IKFast_trans3D(list(pos))
            for i in range(len(sol)):
                # Valid check
                if self.check_legcfg_valid(sol[i]) or not valid_check:
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
            q[3*self.pin_leg_remap[i]:3*self.pin_leg_remap[i]+3] = self.IKFast_foot(i, target_list[i],
                                                                                    valid_check, fall_back, ray_approx)
        return q

    def IK_foots(self, target_list, q0=np.zeros(18)):
        q = q0
        for i in range(6):
            q[3*self.pin_leg_remap[i]:3*self.pin_leg_remap[i] +
                3] = self.IK_foot(i, target_list[i])
        return q
