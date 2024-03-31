#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate, HermiteSpline
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.robot_interface.elspidermini_robotinterface_ros import ElSpiderMini_RobotInterface_ROS, \
    FeetPos2PosList, XYZRPY2SE3, point_SE3Act
from fast_legged_planner_py.swing_leg_planner.cost.cost import CostCollection, KinematicCost, CollisionCost
from fast_legged_planner_py.swing_leg_planner.traj_opt.traj_opt import HermiteOptProb
from fast_legged_planner_py.whole_body_planner.hit_spider_planner_ros import HITSpiderWholeBodyPlanner
from fast_legged_planner.msg import hexapod_State


class ElSpiderAirStateFollower(object):
    def __init__(self) -> None:
        rospy.init_node('elspider_air_swingtraj_planner', anonymous=False)
        self.robot_interface = ElSpiderMini_RobotInterface_ROS(
            rospy.get_param("robot_description"))
        rospy.Subscriber('supportStateTopic', hexapod_State, self.callback)
        self.gridmap_interface = GridMap_Interface("grid_map")
        self.whole_body_planner = HITSpiderWholeBodyPlanner(
            self.gridmap_interface, self.robot_interface)

        self.MCT_solution = []

        # TODO: use speed
        self.rate = 20
        self.ros_rate = rospy.Rate(self.rate)
        # self.base_speed = 0.2
        # self.delta_length = self.base_speed / self.rate

    def callback(self, msg):
        self.MCT_solution.append(msg)
        if msg.remarks.data == "end_flag":
            print("end_flag received, start planning")
            for i in range(len(self.MCT_solution)-1):
                state_0 = self.MCT_solution[i]
                state_1 = self.MCT_solution[i+1]
                self.whole_body_planner.enqueue_MCTsolution(state_0, state_1)
            self.MCT_solution = []
            self.traj_planner()

    def traj_planner(self):
        t = 0  # interpolation parameter
        delta = 0.05  # interpolation step
        while (self.whole_body_planner.get_state_traj_length() > 0):
            state_traj = self.whole_body_planner.get_state_traj(0)
            # World frame
            odom_interp = state_traj.eval_torso_traj(t)
            footend_interp = state_traj.eval_foot_traj(t)
            # Convert to base frame
            for k in range(6):
                footend_interp[k] = point_SE3Act(
                    odom_interp, footend_interp[k])
            # FIXME: This communicates with HLC, whoes frame rate should be stable, continuity be guaranteed
            self.robot_interface.pub_footcmd_from_footendpos(
                footend_interp)
            self.robot_interface.pub_joint_state_from_footendpos(
                footend_interp)
            # World frame
            self.robot_interface.pub_odom(odom_interp)
            # self.robot_interface.pub_foot_trajectory(
            #     self.whole_body_planner.get_foot_traj(t, 80, 0.05))

            t += delta
            if t > 1:
                t = 0
                self.whole_body_planner.dequeue_MCTsolution()
            self.ros_rate.sleep()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    planner = ElSpiderAirStateFollower()
    planner.run()
