#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.robot_interface.elspidermini_robotinterface_ros import ElSpiderMini_RobotInterface_ROS
# from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import cubic_hermite_evaluate, HermiteSpline

if __name__ == '__main__':
    rbt_interface = ElSpiderMini_RobotInterface_ROS(rospy.get_param("robot_description"))
    nh = rospy.init_node('test_elspider_air', anonymous=False)
    rate = rospy.Rate(10)
    foot_end_pos = np.array([[0.2, 0.2, -0.2], [0.2, -0.2, -0.2], [0.2, 0.2, -0.2],
                             [0.2, -0.2, -0.2], [0.2, 0.2, -0.2], [0.2, -0.2, -0.2]])
    while (rospy.is_shutdown() is False):
        rbt_interface.pub_footcmd_from_footendpos(foot_end_pos)
        rate.sleep()
