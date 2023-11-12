#!/usr/bin/env python
# coding=utf-8
from turtle import pos
from typing_extensions import deprecated
import rospy
import tf
import math
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate, linear_evaluate, cubic_evaluate, bezier_evaluate, cubic_bezier_evaluate
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.robot_interface.hitspider_robotinterface import HITSpider_RobotInterface, JOINT_STATE_NAME
from fast_legged_planner.msg import hexapod_State, hexapod_Base_Pose, FeetPosition
from sensor_msgs.msg import JointState


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
    """Convert foot position to footend position

    Args:
        footpos (list): list of foot position

    Returns:
        list: list of footend position
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


class HITSpider_ROS_RobotInterface(HITSpider_RobotInterface):
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

    def pub_joint_state(self, q):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = JOINT_STATE_NAME
        joint_state.position = q
        self.joint_state_pub.publish(joint_state)

    def pub_joint_state_from_footendpos(self, footendpos: list):
        """Publish joint state from footend position

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


class HITSpiderPlanner(object):
    def __init__(self) -> None:
        rospy.init_node('hit_spider_planner', anonymous=False)
        self.robot_interface = HITSpider_ROS_RobotInterface(
            rospy.get_param("robot_description"))
        rospy.Subscriber('supportStateTopic', hexapod_State, self.callback)
        self.gridmap_interface = GridMap_Interface("grid_map")
        self.MCT_solution = []
        self.interp_frame = 100

    def callback(self, msg):
        self.MCT_solution.append(msg)
        if msg.remarks.data == "end_flag":
            rospy.loginfo("接收到消息,质心位置为:%.4f,%.4f,%.4f", msg.base_Pose_Now.position.x,
                          msg.base_Pose_Now.position.y, msg.base_Pose_Now.position.z)
            rospy.loginfo("接收到消息,第一条腿的位置:%.4f,%.4f,%.4f",
                          msg.feetPositionNow.foot[0].x, msg.feetPositionNow.foot[0].y, msg.feetPositionNow.foot[0].z)
            rospy.loginfo("接收到消息,当前腿的支撑状态:%d,%d,%d,%d,%d,%d", msg.support_State_Now[0], msg.support_State_Now[1],
                          msg.support_State_Now[2], msg.support_State_Now[3], msg.support_State_Now[4], msg.support_State_Now[5])
            self.traj_planner()

    def traj_planner(self):
        for i in range(len(self.MCT_solution)-1):
            state_0 = self.MCT_solution[i]
            # FIXME: or use state_next in state_0
            state_1 = self.MCT_solution[i+1]
            for j in range(self.interp_frame):
                odom_interp = self.interp_odom(
                    state_0, state_1, float(j/self.interp_frame))
                footend_interp = self.interp_footend(
                    state_0, state_1, j/self.interp_frame)
                for i in range(6):
                    footend_interp[i] = point_SE3Act(
                        odom_interp, footend_interp[i])
                self.robot_interface.pub_joint_state_from_footendpos(
                    footend_interp)
                self.robot_interface.pub_odom(odom_interp)
                rospy.sleep(1.0/self.interp_frame)

    def interp_footend(self, state_0, state_1, t):
        """
        Interpolate footend between state_0 and state_1

        """
        footend_list0 = FeetPos2PosList(state_0.feetPositionNow)
        footend_list1 = FeetPos2PosList(state_1.feetPositionNow)
        footend_interp = []
        for i in range(6):
            if state_1.support_State_Now[i] == 1:
                footend_interp.append(footend_list0[i])
            else:
                footend_interp.append(self.footend_traj_evaluate(
                    footend_list0[i], footend_list1[i], t))
        return footend_interp

    def footend_traj_evaluate(self, p0, p1, t):
        v = np.array([0, 0, 0.8])
        return cubic_hermite_evaluate(
            np.array([p0, v, p1, -v]), t)

    def interp_odom(self, state_0, state_1, t):
        pose0 = XYZRPY2SE3(state_0.base_Pose_Now)
        pose1 = XYZRPY2SE3(state_1.base_Pose_Now)
        err = pin.log(pose0.actInv(pose1))  # FIXME
        odom_interp = pose0.act(pin.exp(err*t))
        return odom_interp

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = HITSpiderPlanner()
    planner.run()
