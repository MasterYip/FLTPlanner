#!/usr/bin/env python
# coding=utf-8

from typing_extensions import deprecated
import rospy
import tf
import numpy as np
import pinocchio as pin
from .elspidermini_robotinterface import ElSpiderMini_RobotInterface, JOINT_STATE_NAME
from fast_legged_planner.msg import FootCmd
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import JointState
from ..utils.rviz_vis import traj_viz
from ..utils.rviz_vis.traj_viz import TrajViz


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
            '/hexapod/hlc/foot_cmd', FootCmd, queue_size=1)
        self.feedforward_type = 0
        self.joint_kp = 100
        self.joint_kd = 1

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
                Point(footendpos[i][0], footendpos[i][1], footendpos[i][2]))
            footcmd.foot_velocity.append(Vector3(0, 0, 0))
            footcmd.foot_effort.append(Vector3(0, 0, 0))
            footcmd.joint_kp.append(Vector3(self.joint_kp, self.joint_kp, self.joint_kp))
            footcmd.joint_kd.append(Vector3(self.joint_kd, self.joint_kd, self.joint_kd))
            footcmd.joint_torque.append(Vector3(0, 0, 0))
        self.foot_pos_pub.publish(footcmd)
