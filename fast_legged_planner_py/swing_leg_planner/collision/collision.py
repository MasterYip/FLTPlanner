'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-03 21:37:53
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-25 19:26:12
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//collision//collision.py
Description: file content
'''
# -*- coding: utf-8 -*-

# FIXME: This is defined repeatedly
JOINT_STATE_NAME = ["joint_lf_1", "joint_lf_2", "joint_lf_3",
                    "joint_lh_1", "joint_lh_2", "joint_lh_3",
                    "joint_lm_1", "joint_lm_2", "joint_lm_3",
                    "joint_rf_1", "joint_rf_2", "joint_rf_3",
                    "joint_rh_1", "joint_rh_2", "joint_rh_3",
                    "joint_rm_1", "joint_rm_2", "joint_rm_3"]

FOOT_LINK_NAME = ["link_lf_foot", "link_lh_foot", "link_lm_foot",
                  "link_rf_foot", "link_rh_foot", "link_rm_foot"]

class CollisionSphere(object):
    def __init__(self, frame_name, radius):
        self.frame_name = frame_name
        self.radius = radius
    # TODO: What about the sphere which is not at joint center?


class UR5_Collision_Model(object):
    def __init__(self):
        self.collspheres = []
        self.collspheres.append(CollisionSphere("shoulder_pan_joint", 0.1))
        self.collspheres.append(CollisionSphere("shoulder_lift_joint", 0.1))
        self.collspheres.append(CollisionSphere("elbow_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_1_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_2_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_3_joint", 0.08))


class HITSpider_Collision_Model(object):
    def __init__(self):
        self.collspheres = []
        size_map = {"1": 0.15, "2": 0.15, "3": 0.15, "foot": 0.1}
        for name in JOINT_STATE_NAME:
            self.collspheres.append(CollisionSphere(name, size_map[name[-1]]))
        for name in FOOT_LINK_NAME:
            self.collspheres.append(CollisionSphere(name, size_map["foot"]))
