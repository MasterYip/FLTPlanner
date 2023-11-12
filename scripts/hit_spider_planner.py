#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.robot_interface.hitspider_robotinterface_ros import HITSpider_RobotInterface_ROS, \
    FeetPos2PosList, XYZRPY2SE3, point_SE3Act
from fast_legged_planner.msg import hexapod_State


class HITSpiderPlanner(object):
    def __init__(self) -> None:
        rospy.init_node('hit_spider_planner', anonymous=False)
        self.robot_interface = HITSpider_RobotInterface_ROS(
            rospy.get_param("robot_description"))
        rospy.Subscriber('supportStateTopic', hexapod_State, self.callback)
        self.gridmap_interface = GridMap_Interface("grid_map")
        self.MCT_solution = []

        # TODO: use speed
        self.base_speed = 0.2
        self.rate = 30
        self.ros_rate = rospy.Rate(self.rate)
        self.delta_length = self.base_speed / self.rate
        
        # This are auto computed
        self._interp_frame = 20
        self._state_time = 0.5

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
            self.update_state_time(state_0, state_1)

            for j in range(self._interp_frame):
                odom_interp = self.interp_odom(
                    state_0, state_1, float(j/self._interp_frame))
                footend_interp = self.interp_footend(
                    state_0, state_1, j/self._interp_frame)
                for k in range(6):
                    footend_interp[k] = point_SE3Act(
                        odom_interp, footend_interp[k])
                self.robot_interface.pub_joint_state_from_footendpos(
                    footend_interp)
                self.robot_interface.pub_odom(odom_interp)
                self.robot_interface.pub_foot_trajectory(
                    self.get_interp_foottraj(self.MCT_solution[i:i+8]))
                self.ros_rate.sleep()

    def get_interp_foottraj(self, state_list):
        # TrajViz
        foot_traj_list = [[] for _ in range(6)]
        for i in range(len(state_list)-1):
            for j in range(self._interp_frame):
                foot_pos_list = self.interp_footend(
                    state_list[i], state_list[i+1], j/self._interp_frame)
                for k in range(6):
                    foot_traj_list[k].append(foot_pos_list[k])
        return foot_traj_list

    def interp_footend(self, state_0, state_1, t):
        """
        Interpolate foot position between state_0 and state_1

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

    def update_state_time(self, state_0, state_1):
        pose0 = XYZRPY2SE3(state_0.base_Pose_Now)
        pose1 = XYZRPY2SE3(state_1.base_Pose_Now)
        dis = np.linalg.norm(pose0.translation - pose1.translation)
        self._state_time = dis / self.base_speed
        self._interp_frame = int(self._state_time * self.rate)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = HITSpiderPlanner()
    planner.run()
