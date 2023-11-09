#!/usr/bin/env python
# coding=utf-8
import rospy
import tf
import math
import numpy as np
import fast_legged_planner_py
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import linear_evaluate, cubic_evaluate, bezier_evaluate, cubic_bezier_evaluate
# 导入mgs
from fast_legged_planner.msg import hexapod_State, hexapod_Base_Pose
from sensor_msgs.msg import JointState


class HITSpiderPlanner(object):
    def __init__(self) -> None:
        rospy.init_node('hit_spider_planner', anonymous=False)
        rospy.Subscriber('supportStateTopic', hexapod_State, self.callback)
        self.joint_state_pub = rospy.Publisher(
            'joint_state', JointState, queue_size=10)
        self.odom_pub = tf.TransformBroadcaster()
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
        for i in range(len(self.MCT_solution)):
            state_0 = self.MCT_solution[i]
            # FIXME: or use state_next in state_0
            state_1 = self.MCT_solution[i+1]
            for j in range(self.interp_frame):
                # state_interp = self.interp_joint_state(state, j/self.interp_frame)
                odom_interp = self.interp_odom(
                    state_0, state_1, float(j/self.interp_frame))
                # self.joint_state_pub.publish(state_interp)
                self.odom_pub.sendTransform((odom_interp.position.x, odom_interp.position.y, odom_interp.position.z),
                                            tf.transformations.quaternion_from_euler(
                                                odom_interp.orientation.roll, odom_interp.orientation.pitch, odom_interp.orientation.yaw),
                                            rospy.Time.now(),
                                            "link_base",
                                            "odom")
                rospy.sleep(1.0/self.interp_frame)

    def interp_odom(self, state_0, state_1, t):
        odom_interp = hexapod_Base_Pose()
        odom_interp.position.x = linear_evaluate(
            np.array([state_0.base_Pose_Now.position.x, state_1.base_Pose_Now.position.x]), t)
        odom_interp.position.y = linear_evaluate(
            np.array([state_0.base_Pose_Now.position.y, state_1.base_Pose_Now.position.y]), t)
        odom_interp.position.z = linear_evaluate(
            np.array([state_0.base_Pose_Now.position.z, state_1.base_Pose_Now.position.z]), t)

        odom_interp.orientation.roll = linear_evaluate(
            np.array([state_0.base_Pose_Now.orientation.roll, state_1.base_Pose_Now.orientation.roll]), t)
        odom_interp.orientation.pitch = linear_evaluate(
            np.array([state_0.base_Pose_Now.orientation.pitch, state_1.base_Pose_Now.orientation.pitch]), t)
        odom_interp.orientation.yaw = linear_evaluate(
            np.array([state_0.base_Pose_Now.orientation.yaw, state_1.base_Pose_Now.orientation.yaw]), t)
        return odom_interp

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = HITSpiderPlanner()
    planner.run()
