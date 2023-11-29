'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-03 21:37:53
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-29 21:08:33
FilePath: //flplanner_ws//src//fast_legged_planner//fast_legged_planner_py//swing_leg_planner//collision//collision.py
Description: file content
'''
# -*- coding: utf-8 -*-
from ...third_party.meshcat_viewer_wrapper.visualizer import colors
from ...utils.rviz_vis.traj_viz import COLOR_GREEN, COLOR_RED, get_scale_vector3

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

    def getCollCost(self, sdf_value):
        c = self.radius - sdf_value
        cost = 0
        if c > self.radius:
            cost = c**2 + c
        elif c > 0:
            cost = c
        return cost


class UR5_Collision_Model(object):
    def __init__(self):
        self.collspheres = []
        self.collspheres.append(CollisionSphere("shoulder_pan_joint", 0.1))
        self.collspheres.append(CollisionSphere("shoulder_lift_joint", 0.1))
        self.collspheres.append(CollisionSphere("elbow_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_1_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_2_joint", 0.08))
        self.collspheres.append(CollisionSphere("wrist_3_joint", 0.08))


class HITLeg_Collision_Model(object):
    def __init__(self, robot_interface, map_interface, leg_index):
        self.robot_interface = robot_interface
        self.map_interface = map_interface
        self.leg_index = leg_index

        size_map = {"1": 0.15, "2": 0.15, "3": 0.15, "foot": 0.02}
        self.collspheres = []
        for name in JOINT_STATE_NAME[self.leg_index*3:self.leg_index*3+3]:
            self.collspheres.append(CollisionSphere(name, size_map[name[-1]]))
        self.collspheres.append(CollisionSphere(
            FOOT_LINK_NAME[self.leg_index], size_map["foot"]))

    def checkCollision(self, q_leg, pose_base):
        """Check if the leg is in collision
        :param q_leg: leg joint angles
        :param pose_base: base pose
        """
        q = self.robot_interface.get_full_q(q_leg, self.leg_index)
        self.robot_interface.update_kinematics(q)
        for sphere in self.collspheres:
            # placement under base frame
            m = self.robot_interface.get_frame_placement(
                q, sphere.frame_name, update_kinematics=False)
            # FIXME Is this correct?
            m_world = pose_base * m
            p = m_world.translation
            sdf_value = self.map_interface.sdf_value(p)
            if sdf_value < sphere.radius:
                return True
        return False

    def getCollCost(self, q_leg, pose_base):
        """Compute the collision cost
        :param q_leg: leg joint angles
        :param pose_base: base pose
        """
        cost = 0
        q = self.robot_interface.get_full_q(q_leg, self.leg_index)
        self.robot_interface.update_kinematics(q)
        for sphere in self.collspheres:
            # placement under base frame
            m = self.robot_interface.get_frame_placement(
                q, sphere.frame_name, update_kinematics=False)
            # FIXME Is this correct?
            m_world = pose_base * m
            p = m_world.translation
            sdf_value = self.map_interface.sdf_value(p)
            cost += sphere.getCollCost(sdf_value)
        return cost

    def getCollCost_IK(self, pos_foot, pose_base):
        """Compute the collision cost
        :param pos_foot: foot position in world frame
        :param pose_base: base pose
        """
        pos_foot_base = pose_base.inverse() * pos_foot
        q_leg = self.robot_interface.IKFast_foot(self.leg_index, pos_foot_base, valid_check=False)
        # return self.collspheres[-1].getCollCost(self.map_interface.sdf_value(pos_foot))
        return self.getCollCost(q_leg, pose_base)

    def vis_collision_model(self, q_leg, type="meshcat", base_pose=None):
        q = self.robot_interface.get_full_q(q_leg, self.leg_index)
        if type == "meshcat":
            for collsphere in self.collspheres:
                viz_id = "world/collsphere/"+collsphere.frame_name
                self.viz.addSphere(
                    viz_id, collsphere.radius, colors.green_transparent)
                self.viz.applyConfiguration(
                    viz_id, self.robot_interface.get_frame_placement(q, collsphere.frame_name))
        elif type == "Rviz" and base_pose is not None:
            for collsphere in self.collspheres:
                self.robot_interface.traj_viz.add_spheres(
                    base_pose.act(self.robot_interface.get_frame_placement(
                        q, collsphere.frame_name)),
                    scale=get_scale_vector3([1, 1, 1]*collsphere.radius),
                    color=colors.green_transparent)
            self.robot_interface.traj_viz.publish()
        else:
            raise ValueError(
                "Supported type: meshcat, Rviz. Rviz type need base_pose")
        


# class HITSpider_Collision_Model(object):
#     def __init__(self, robot_interface, map_interface):
#         self.robot_interface = robot_interface
#         self.map_interface = map_interface

#         self.collspheres = []
#         size_map = {"1": 0.15, "2": 0.15, "3": 0.15, "foot": 0.1}
#         for name in JOINT_STATE_NAME:
#             self.collspheres.append(CollisionSphere(name, size_map[name[-1]]))
#         for name in FOOT_LINK_NAME:
#             self.collspheres.append(CollisionSphere(name, size_map["foot"]))
