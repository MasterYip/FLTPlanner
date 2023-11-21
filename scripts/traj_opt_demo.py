'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-20 16:12:38
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-21 16:34:26
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
from fast_legged_planner_py.utils.rviz_vis.traj_viz import TrajViz, COLOR_GREEN, COLOR_RED, SCALE_MEDIUM, SCALE_LARGE


def get_1stage_traj():
    p_start = np.array([-1, 0, 0])
    p_end = np.array([1, 0, 0])
    v = np.array([0, 0, 0.8])
    return HermiteSpline(
        np.array([p_start, v, p_end, -v]))


def get_2stage_traj():
    p_start = np.array([-1, 0, 0])
    p_end = np.array([1, 0, 0])
    v = np.array([0, 0, 0.8])
    p_mid = (p_start+p_end)*0.5
    v_mid = (p_end-p_start)*0.5
    return HermiteSpline(
        np.array([p_start, v, p_mid, v_mid, p_end, -v]))


class TrajOptDemo(object):
    def __init__(self) -> None:
        # ROS init
        rospy.init_node("traj_opt_demo")
        self.rate = rospy.Rate(2)
        # Config Spline
        self.spline = get_1stage_traj()
        # self.spline.insert_normalized(np.linspace(0, 1, 10)[1:-1])
        # Visualization
        self.resolution = 200
        self.traj_viz = TrajViz("traj_opt", "odom")
        # Gridmap
        self.map_interface = GridMap_Interface()
        self.map_interface.update()

    def optimize_viz(self, maxiter=100):
        cnt = 0
        self.costs = CostCollection([
            KinematicCost(self.spline, 0.3),
            CollisionCost(self.spline, self.map_interface, 10)
        ])
        self.prob = TrajOptProblem(self.spline, self.costs, None,
                                   self.spline.get())
        while (not rospy.is_shutdown() and cnt < maxiter):
            self.viz_traj()
            self.prob.optimize(maxiter=1)
            self.rate.sleep()
            cnt += 1
            rospy.loginfo("Optimize %d times" % cnt)

    def viz_traj(self):
        self.traj_viz.add_curve([self.spline.evaluate_normalized(t)
                                 for t in np.linspace(0, 1, self.resolution)],
                                color=COLOR_GREEN, linewidth=0.02)
        self.traj_viz.add_spheres(
            self.spline.get_poslist(), color=COLOR_GREEN, scale=SCALE_MEDIUM)
        self.traj_viz.publish()


if __name__ == "__main__":
    demo = TrajOptDemo()
    demo.viz_traj()
    rospy.sleep(1)
    demo.spline.insert_normalized(np.linspace(0, 1, 5)[1:-1])
    demo.viz_traj()
    demo.optimize_viz()
    rospy.spin()
