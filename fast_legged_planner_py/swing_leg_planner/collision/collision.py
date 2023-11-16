'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-03 21:37:53
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-04 09:33:17
FilePath: /fast_legged_planner/fast_legged_planner_py/swing_leg_planner/collision/collision.py
Description: file content
'''
# -*- coding: utf-8 -*-


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
