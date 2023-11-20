'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-20 16:12:38
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-20 17:24:47
FilePath: //flplanner_ws//src//fast_legged_planner//scripts//traj_opt_demo.py
Description: file content
'''

#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import HermiteSpline
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.swing_leg_planner.swing_traj_planner import SwingTrajPlanner
from fast_legged_planner_py.swing_leg_planner.traj_opt.traj_opt import TrajOptProblem
from fast_legged_planner_py.swing_leg_planner.cost.cost import CostCollection, KinematicCost, CollisionCost
from fast_legged_planner_py.utils.rviz_vis.traj_viz import TrajViz, COLOR_GREEN


class TrajOptDemo(object):
    def __init__(self) -> None:
        # ROS init
        rospy.init_node("traj_opt_demo")
        self.rate = rospy.Rate(5)
        # Interface
        self.map_interface = GridMap_Interface()
        self.map_interface.update()
        self.traj_viz = TrajViz("traj_opt", "odom")
        # Config
        self.p_start = np.array([-1, 0, 0])
        self.p_end = np.array([1, 0, 0])

        v = np.array([0, 0, 0.8])
        p_mid = (self.p_start+self.p_end)*0.5
        v_mid = (self.p_end-self.p_start)*0.5
        self.spline = HermiteSpline(
            np.array([self.p_start, v, p_mid, v_mid, self.p_end, -v]))

        self.resolution = 50

    def optimize_viz(self, maxiter=100):
        cnt = 0
        self.costs = CostCollection([
            KinematicCost(self.spline, 0.3),
            CollisionCost(self.spline, self.map_interface, 10)
        ])
        self.prob = TrajOptProblem(self.spline, self.costs, None,
                                   self.spline.knots)
        while (not rospy.is_shutdown() and cnt < maxiter):
            self.traj_viz.add_curve([self.spline.evaluate_normalized(t)
                                     for t in np.linspace(0, 1, self.resolution)],
                                    color=COLOR_GREEN, linewidth=0.05)
            self.traj_viz.publish()
            self.prob.optimize(maxiter=1)
            self.rate.sleep()
            cnt += 1
            rospy.loginfo("Optimize %d times" % cnt)


if __name__ == "__main__":
    demo = TrajOptDemo()
    rospy.sleep(2)
    demo.optimize_viz()
