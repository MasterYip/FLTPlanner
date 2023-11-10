'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-09 21:32:46
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 22:43:52
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/hitspider_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-


import numpy as np
from .base_robotinterface import Base_RobotInterface
# from .pin_IK import pinIK
from scipy.optimize import fmin_bfgs
from numpy.linalg import norm

JOINT_STATE_NAME = ["joint_lf_1", "joint_lf_2", "joint_lf_3",
                    "joint_lh_1", "joint_lh_2", "joint_lh_3",
                    "joint_lm_1", "joint_lm_2", "joint_lm_3",
                    "joint_rf_1", "joint_rf_2", "joint_rf_3",
                    "joint_rh_1", "joint_rh_2", "joint_rh_3",
                    "joint_rm_1", "joint_rm_2", "joint_rm_3"]

FOOT_LINK_NAME = ["link_lf_foot", "link_lh_foot", "link_lm_foot",
                  "link_rf_foot", "link_rh_foot", "link_rm_foot"]


class HITSpider_RobotInterface(Base_RobotInterface):
    def __init__(self, urdf: str, package_dirs=None) -> None:
        super().__init__(urdf, package_dirs)

    def get_full_q(self, q_leg, footlink_num):
        q = np.zeros(18)
        index = footlink_num*3
        q[index:index+3] = q_leg
        return q

    def IK_foot(self, footlink_num, target, q_leg0=np.zeros(3)):
        """Get the target configuration

        Args:
            target (np.ndarray(3,)): target 3d position relative to base
            q0 (np.ndarray(3,), optional): last config, to speed up solver.
        """
        def cost(q_leg):
            '''Compute score from a configuration'''
            m = self.get_frame_placement(self.get_full_q(
                q_leg, footlink_num), FOOT_LINK_NAME[footlink_num])
            p = m.translation
            # offset = m.rotation[:, 2] * radius
            # return norm(p +  offset - target)**2
            return norm(p - target)**2

        return fmin_bfgs(cost, q_leg0, disp=False, gtol=1e-4)

    def IK_foots(self, target_list, q0=np.zeros(18)):
        q = q0
        for i in range(6):
            q[3*i:3*i+3] = self.IK_foot(i, target_list[i])
        return q
