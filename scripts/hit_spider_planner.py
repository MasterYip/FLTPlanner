#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate, HermiteSpline
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.robot_interface.hitspider_robotinterface_ros import HITSpider_RobotInterface_ROS, \
    FeetPos2PosList, XYZRPY2SE3, point_SE3Act
from fast_legged_planner_py.swing_leg_planner.cost.cost import CostCollection, KinematicCost, CollisionCost
from fast_legged_planner_py.swing_leg_planner.traj_opt.traj_opt import HermiteOptProb
from fast_legged_planner_py.whole_body_planner.hit_spider_planner_ros import HITSpiderWholeBodyPlanner
from fast_legged_planner.msg import hexapod_State


class HITSpiderPlanner(object):
    def __init__(self) -> None:
        rospy.init_node('hit_spider_planner', anonymous=False)
        self.robot_interface = HITSpider_RobotInterface_ROS(
            rospy.get_param("robot_description"))
        rospy.Subscriber('supportStateTopic', hexapod_State, self.callback)
        self.gridmap_interface = GridMap_Interface("grid_map")
        self.whole_body_planner = HITSpiderWholeBodyPlanner(
            self.gridmap_interface, self.robot_interface)

        self.MCT_solution = []

        # TODO: use speed
        self.base_speed = 0.2
        self.rate = 30
        self.ros_rate = rospy.Rate(self.rate)
        self.delta_length = self.base_speed / self.rate

        # This are auto computed
        self._interp_frame = 20
        self._state_time = 0.5

        # Optimized data
        self.opt_traj = None
        self.opt_state = None  # used for opt_traj validation

    def callback(self, msg):
        self.MCT_solution.append(msg)
        if msg.remarks.data == "end_flag":
            rospy.loginfo("接收到消息,质心位置为:%.4f,%.4f,%.4f", msg.base_Pose_Now.position.x,
                          msg.base_Pose_Now.position.y, msg.base_Pose_Now.position.z)
            rospy.loginfo("接收到消息,第一条腿的位置:%.4f,%.4f,%.4f",
                          msg.feetPositionNow.foot[0].x, msg.feetPositionNow.foot[0].y, msg.feetPositionNow.foot[0].z)
            rospy.loginfo("接收到消息,当前腿的支撑状态:%d,%d,%d,%d,%d,%d", msg.support_State_Now[0], msg.support_State_Now[1],
                          msg.support_State_Now[2], msg.support_State_Now[3], msg.support_State_Now[4], msg.support_State_Now[5])
            for i in range(len(self.MCT_solution)-1):
                state_0 = self.MCT_solution[i]
                state_1 = self.MCT_solution[i+1]
                self.whole_body_planner.enqueue_MCTsolution(state_0, state_1)
            self.traj_planner()

    def traj_planner(self):
        t = 0
        delta = 0.05
        while (self.whole_body_planner.get_state_traj_length() > 0):
            state_traj = self.whole_body_planner.get_state_traj(0)

            odom_interp = state_traj.eval_torso_traj(t)
            footend_interp = state_traj.eval_foot_traj(t)
            for k in range(6):
                footend_interp[k] = point_SE3Act(
                    odom_interp, footend_interp[k])
            self.robot_interface.pub_joint_state_from_footendpos(
                footend_interp)
            self.robot_interface.pub_odom(odom_interp)
            self.robot_interface.pub_foot_trajectory(
                self.whole_body_planner.get_foot_traj(t, 80, 0.05))

            t += delta
            if t > 1:
                t = 0
                self.whole_body_planner.dequeue_MCTsolution()
            self.ros_rate.sleep()

        # for i in range(len(self.MCT_solution)-1):
        #     state_0 = self.MCT_solution[i]
        #     # FIXME: or use state_next in state_0
        #     state_1 = self.MCT_solution[i+1]
        #     self.whole_body_planner.enqueue_MCTsolution(state_0, state_1)
        #     self.update_state_time(state_0, state_1)

        #     for j in range(self._interp_frame):
        #         odom_interp = self.interp_odom(
        #             state_0, state_1, float(j/self._interp_frame))
        #         footend_interp = self.interp_footend(
        #             state_0, state_1, j/self._interp_frame)
        #         for k in range(6):
        #             footend_interp[k] = point_SE3Act(
        #                 odom_interp, footend_interp[k])
        #         self.robot_interface.pub_joint_state_from_footendpos(
        #             footend_interp)
        #         self.robot_interface.pub_odom(odom_interp)
        #         self.robot_interface.pub_foot_trajectory(
        #             self.get_interp_foottraj(self.MCT_solution[i:i+8]))
        #         self.ros_rate.sleep()

    # def update_state_time(self, state_0, state_1):
    #     pose0 = XYZRPY2SE3(state_0.base_Pose_Now)
    #     pose1 = XYZRPY2SE3(state_1.base_Pose_Now)
    #     dis = np.linalg.norm(pose0.translation - pose1.translation)
    #     self._state_time = dis / self.base_speed
    #     self._interp_frame = int(self._state_time * self.rate)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = HITSpiderPlanner()
    planner.run()
