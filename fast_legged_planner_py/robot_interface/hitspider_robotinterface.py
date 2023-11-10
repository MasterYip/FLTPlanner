'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-09 21:32:46
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 14:15:54
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/hitspider_robotinterface.py
Description: file content
'''
# -*- coding: utf-8 -*-


import numpy as np
from .base_robotinterface import Base_RobotInterface

# import rospy
# import tf
# # 导入mgs
# from fast_legged_planner.msg import hexapod_State, hexapod_Base_Pose
# from sensor_msgs.msg import JointState

HEXAPOD_JOINT_STATE_NAME = ["joint_lf_1", "joint_lf_2", "joint_lf_3",
                            "joint_lm_1", "joint_lm_2", "joint_lm_3",
                            "joint_lh_1", "joint_lh_2", "joint_lh_3",
                            "joint_rf_1", "joint_rf_2", "joint_rf_3",
                            "joint_rm_1", "joint_rm_2", "joint_rm_3",
                            "joint_rh_1", "joint_rh_2", "joint_rh_3"]


class HITSpider_RobotInterface(Base_RobotInterface):
    def __init__(self, urdf: str, package_dirs=None) -> None:
        super().__init__(urdf, package_dirs)

        
