#!/usr/bin/env python
# coding=utf-8

from typing_extensions import deprecated
import rospy
import tf
import numpy as np
import pinocchio as pin
from .elspidermini_robotinterface import ElSpiderMini_RobotInterface, JOINT_STATE_NAME
from fast_legged_planner.msg import hexapod_Base_Pose, FeetPosition
from fast_legged_planner.msg import FootCmd
from geometry_msgs.msg import Point, Vector3
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


class ElSpiderMini_RobotInterface_ROS(ElSpiderMini_RobotInterface):

    def __init__(self, urdf: str) -> None:
        """
        :param urdf: URDF file path OR URDF string
        """
        super().__init__(urdf)
        # Publish joint state & odom
        self.joint_state_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)
        self.odom_pub = tf.TransformBroadcaster()
        self.traj_viz = TrajViz("elspiderair_expected_traj", "odom")
        # Publisher&Params for HexapodSoftware High Level Controller
        self.foot_pos_pub = rospy.Publisher(
            '/hexapod/hlc/foot_cmd_track', FootCmd, queue_size=1)
        self.feedforward_type = 0
        # Hardware
        self.joint_kp = [0.075, 0.2, 0.2]
        self.joint_kd = [2, 2, 2]
        # self.joint_kp = [0.025, 0.05, 0.05]
        # self.joint_kd = [1, 1, 1]
        # Gazebo
        # self.joint_kp = [500, 500, 500]
        # self.joint_kd = [3, 7.5, 7.5]

    # for HexapodSoftware High Level Controller
    def pub_footcmd_from_footendpos(self, footendpos: list):
        """Publish FootCmd from foot position to HexapodSoftware High Level Controller

        Args:
            footendpos (list): Under base frame
        """
        footcmd = FootCmd()
        footcmd.header.stamp = rospy.Time.now()
        footcmd.feedforward_type = self.feedforward_type
        for i in range(6):
            footcmd.foot_position.append(
                Point(footendpos[i][0],
                      footendpos[i][1],
                      footendpos[i][2]))
            footcmd.foot_velocity.append(Vector3(0, 0, 0))
            footcmd.foot_effort.append(Vector3(0, 0, 0))
            footcmd.joint_kp.append(
                Vector3(self.joint_kp[0], self.joint_kp[1], self.joint_kp[2]))
            footcmd.joint_kd.append(
                Vector3(self.joint_kd[0], self.joint_kd[1], self.joint_kd[2]))
            footcmd.joint_torque.append(Vector3(0, 0, 0))
        self.foot_pos_pub.publish(footcmd)

    def pub_odom(self, odom: pin.SE3()):
        xyzq = pin.SE3ToXYZQUAT(odom)
        self.odom_pub.sendTransform(xyzq[:3],   # xyz
                                    xyzq[3:],   # quat
                                    rospy.Time.now(),
                                    "base",
                                    "odom")

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
        self.pub_joint_state(self.IKFast_foots(
            [footendpos[i] for i in range(6)]))
