#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import copy
import pinocchio as pin
from fast_legged_planner_py.robot_interface.elspidermini_robotinterface_ros import ElSpiderMini_RobotInterface_ROS
# from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate, HermiteSpline

if __name__ == '__main__':
    rbt_interface = ElSpiderMini_RobotInterface_ROS(
        rospy.get_param("robot_description"))
    nh = rospy.init_node('test_elspider_air', anonymous=False)
    freq = 100
    rate = rospy.Rate(freq)
    t = 0
    foot_end_pos = np.array([[0.35350208, -0.22998902, -0.1360558],
                             [0.05350208, -0.28998902, -0.1360558],
                             [-0.35349792, -0.22998928, -0.1360562],
                             [0.35350208,  0.2299882, -0.1360569],
                             [0.05350208,  0.2899882, -0.1360569],
                             [-0.35350133, 0.22998794, -0.13605704]])

    while (rospy.is_shutdown() is False):
        tmp_pos = copy.deepcopy(foot_end_pos)
        tmp_pos[0] += np.array([0, 0, 0.1])*np.sin(t)
        rbt_interface.pub_footcmd_from_footendpos(tmp_pos)
        rate.sleep()
        t += 1./freq
