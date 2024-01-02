#!/usr/bin/env python
# coding=utf-8

from typing_extensions import deprecated
import rospy
import tf
import numpy as np
import pinocchio as pin
from .hitspider_robotinterface import HITSpider_RobotInterface, JOINT_STATE_NAME
from fast_legged_planner.msg import hexapod_Base_Pose, FeetPosition
from sensor_msgs.msg import JointState
from ..utils.rviz_vis import traj_viz
from ..utils.rviz_vis.traj_viz import TrajViz


def XYZRPY2SE3(pose: hexapod_Base_Pose):
    """Convert Pose(xyzrpy) to SE3

    Args:
        pose (hexapod_Base_Pose): pose XYZRPY

    Returns:
        pin.SE3: SE3
    """
    # RPY to Rotation Matrix
    return pin.SE3(pin.rpy.rpyToMatrix(np.array([pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw])),
                   np.array([pose.position.x, pose.position.y, pose.position.z]))


def FeetPos2PosList(footpos: FeetPosition):
    """Convert msg FeetPosition to foot position

    Args:
        footpos (list): list of foot position

    Returns:
        list: list of foot position
    """
    footendpos = []
    for i in range(6):
        footendpos.append(
            np.array([footpos.foot[i].x, footpos.foot[i].y, footpos.foot[i].z]))
    return footendpos


def point_SE3Act(bMa: pin.SE3, pt: np.ndarray):
    """Act on point

    Args:
        bMa (pin.SE3): SE3
        pt (np.ndarray): point

    Returns:
        np.ndarray: point
    """
    aMb = bMa.inverse()
    return aMb.translation + aMb.rotation @ pt


class HITSpider_RobotInterface_ROS(HITSpider_RobotInterface):
    footend_order = [0, 2, 1, 5, 3, 4]

    def __init__(self, urdf: str) -> None:
        """
        :param urdf: URDF file path OR URDF string
        """
        super().__init__(urdf)
        # Publish joint state & odom
        self.joint_state_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)
        self.odom_pub = tf.TransformBroadcaster()
        self.traj_viz = TrajViz("hit_spider_expected_traj", "odom")

    def pub_joint_state(self, q):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = JOINT_STATE_NAME
        joint_state.position = q
        self.joint_state_pub.publish(joint_state)

    def pub_joint_state_from_footendpos(self, footendpos: list):
        """Publish joint state from foot position

        Args:
            footendpos (list): Under base frame
        """
        self.pub_joint_state(self.IK_foots(
            [footendpos[self.footend_order[i]] for i in range(6)]))

    def pub_odom(self, odom: pin.SE3()):
        xyzq = pin.SE3ToXYZQUAT(odom)
        self.odom_pub.sendTransform(xyzq[:3],   # xyz
                                    xyzq[3:],   # quat
                                    rospy.Time.now(),
                                    "link_base",
                                    "odom")

    @deprecated
    def pub_odom_from_pose(self, odom: hexapod_Base_Pose):
        self.odom_pub.sendTransform((odom.position.x, odom.position.y, odom.position.z),
                                    tf.transformations.quaternion_from_euler(
                                    odom.orientation.roll, odom.orientation.pitch, odom.orientation.yaw),
                                    rospy.Time.now(),
                                    "link_base",
                                    "odom")

    def pub_foot_trajectory(self, foot_traj_list: tuple):
        """Publish foot trajectory

        Args:
            foot_traj_list (default, optimized): list of foot trajectory(list of list of points)
        """
        for traj in foot_traj_list[0]:
            self.traj_viz.add_curve(traj, "foot_traj", traj_viz.COLOR_RED)
        for traj in foot_traj_list[1]:
            self.traj_viz.add_curve(traj, "foot_traj", traj_viz.COLOR_GREEN)
        self.traj_viz.publish()
