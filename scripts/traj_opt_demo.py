'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-20 16:12:38
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-30 14:17:31
FilePath: //flplanner_ws//src//fast_legged_planner//scripts//traj_opt_demo.py
Description: file content
'''

#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import numpy as np
import pinocchio as pin
from fast_legged_planner_py.robot_interface.hitspider_robotinterface_ros import HITSpider_RobotInterface_ROS
from fast_legged_planner_py.swing_leg_planner.traj_gen.traj_gen import HermiteSpline, UniBSpline, TimedLinearSpline
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface
from fast_legged_planner_py.swing_leg_planner.swing_traj_planner import SwingTrajPlanner
from fast_legged_planner_py.swing_leg_planner.traj_opt.traj_opt import HermiteOptProb, UniBSplineOptProb, Legged_UniBSplineOptProb, RRTBSplineOptProb, RRTCfg_BSplineOptProb
from fast_legged_planner_py.swing_leg_planner.cost.cost import CostCollection, KinematicCost, CollisionCost
from fast_legged_planner_py.utils.rviz_vis.traj_viz import TrajViz, COLOR_GREEN, COLOR_RED, SCALE_MEDIUM, SCALE_LARGE
from fast_legged_planner_py.swing_leg_planner.traj_opt.rrt.rrt_interface import Gridmap_SearchSpace
from fast_legged_planner_py.third_party.rrt_algorithms.src.rrt.rrt_connect import RRTConnect
from fast_legged_planner_py.third_party.rrt_algorithms.src.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from fast_legged_planner_py.third_party.rrt_algorithms.src.utilities.plotting import Plot

# Hermite


def get_1stage_traj():
    p_start = np.array([-0.5, 0, 0])
    p_end = np.array([0.5, 0, 0])
    v = np.array([0, 0, 1])
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

# B-Spline


def get_bspline():
    hermite = get_1stage_traj()
    resolution = 5
    return UniBSpline(np.array([hermite.evaluate(t, normalized=True) for t in np.linspace(0, 1, resolution)]), 3)


def torso_traj(t):
    pose0 = pin.XYZQUATToSE3([-0.5, 1, 0.5, 0, 0, 0, 1])
    pose1 = pin.XYZQUATToSE3([0.5, 1, 0.5, 0, 0, 0, 1])
    err = pin.log(pose0.actInv(pose1))  # FIXME
    odom_interp = pose0.act(pin.exp(err*t))
    return odom_interp


class TrajOptDemo(object):
    def __init__(self) -> None:
        # ROS init
        rospy.init_node("traj_opt_demo")
        self.rate = rospy.Rate(5)
        # Config Spline
        self.spline = get_bspline()
        # self.spline.insert_normalized(np.linspace(0, 1, 10)[1:-1])

        # Visualization
        self.resolution = 200
        self.traj_viz = TrajViz("traj_opt", "odom")
        # Gridmap
        self.map_interface = GridMap_Interface("grid_map")
        self.map_interface.update()
        # Robot
        self.with_robot = rospy.get_param("traj_opt_demo/with_robot", False)
        if self.with_robot:
            self.leg_index = 1
            self.robot_interface = HITSpider_RobotInterface_ROS(
                rospy.get_param("robot_description"))
            self.robot_interface.pub_joint_state(self.robot_interface.robot.q0)

    # BSpline TrajOpt
    def optimize_viz(self, maxiter=10):
        cnt = 0
        # self.costs = CostCollection([
        #     KinematicCost(self.spline, 0.3),
        #     CollisionCost(self.spline, self.map_interface, 10)
        # ])
        # self.prob = HermiteOptProb(self.spline, self.costs, None,
        #                            self.spline.get())
        self.prob = UniBSplineOptProb(self.spline, self.map_interface)
        while self.prob.get_max_collision_index() is not None and rospy.is_shutdown() is False:
            if cnt > 0:
                self.spline.insert(
                    self.prob.get_max_collision_index(), normalized=True)
                rospy.logwarn("Insert knot at %f for further opt." %
                              self.prob.get_first_collision_index())
            cnt = 0
            while (not rospy.is_shutdown() and cnt < maxiter):
                self.viz_traj()
                self.prob.optimize(maxiter=1, use_fprime=False, disp=False)
                self.rate.sleep()
                cnt += 1
                rospy.loginfo("Optimize %d times" % cnt)

    def viz_traj(self):
        self.traj_viz.add_curve([self.spline.evaluate(t, normalized=True)
                                 for t in np.linspace(0, 1, self.resolution)],
                                color=COLOR_GREEN, linewidth=0.02)
        self.traj_viz.add_spheres(
            self.spline.get_poslist(), color=COLOR_GREEN, scale=SCALE_MEDIUM)
        self.traj_viz.publish()

    # RRT TrajOpt
    def optimize_viz_rrt(self, webplot=False):
        x_init = (-1.0, 0., 0.)  # starting location
        x_goal = (1.0, 0., 0.)  # goal location

        # RRT_Connect
        # Q = np.array([0.3])  # length of tree edges
        # r = 0.05  # length of smallest edge to check for intersection with obstacles
        # max_samples = 9024  # max number of samples to take before timing out
        # prc = 0.1  # probability of checking for a connection to goal
        # X = Gridmap_SearchSpace(self.map_interface)
        # rrt_connect = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)
        # path = rrt_connect.rrt_connect(verbose=False)
        # self.rrt_viz_traj(path)
        # self.rrt_webplot(X, x_init, x_goal, rrt_connect, path=path)

        # RRT*_Connect_h
        Q = np.array([(0.1, 4)])  # length of tree edges
        r = 0.01  # length of smallest edge to check for intersection with obstacles
        max_samples = 1024  # max number of samples to take before timing out
        rewire_count = 32  # optional, number of nearby branches to rewire
        prc = 0.01  # probability of checking for a connection to goal

        X = Gridmap_SearchSpace(self.map_interface, x_init, x_goal)
        rrt_star_bid_h = RRTStarBidirectionalHeuristic(
            X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path = rrt_star_bid_h.rrt_star_bid_h(verbose=False)
        # self.rrt_viz_traj(path)
        timed_path = []
        for i in range(len(path)):
            t = [i/len(path)] + list(path[i])
            timed_path.append(np.array(t))
        self.spline = TimedLinearSpline(np.array(timed_path))
        self.viz_traj()
        if webplot:
            self.rrt_webplot(X, x_init, x_goal, rrt_star_bid_h, path=path)

    def rrt_viz_traj(self, path):
        if path is not None:
            self.traj_viz.add_curve(path, color=COLOR_RED, linewidth=0.02)
            self.traj_viz.publish()

    def rrt_webplot(self, X, x_init, x_goal, rrt, path=None):
        # plot
        plot = Plot("rrt_connect_3d")
        plot.plot_tree(X, rrt.trees)
        if path is not None:
            plot.plot_path(X, path)
        # plot.plot_obstacles(X, Obstacles)
        plot.plot_start(X, x_init)
        plot.plot_goal(X, x_goal)
        plot.draw(auto_open=True)

    # RRT Cfg TrajOpt
    def optimize_viz_rrt_cfg(self):
        cnt = 0
        prob = RRTCfg_BSplineOptProb(self.spline, torso_traj, self.leg_index,
                                     self.map_interface, self.robot_interface,
                                     end_ignore_dia=0.07)
        self.spline = prob.optimize(Q=np.array([[0.05, 4]]), max_samples=1024)


    # Robot Viz

    def viz_robot_traj(self):
        res = 100
        for t in np.linspace(0, 1, res):
            pos = self.spline.evaluate(t, normalized=True)
            pos_base = torso_traj(t).actInv(pos)
            self.robot_interface.pub_joint_state(
                self.robot_interface.get_full_q(
                    self.robot_interface.IKFast_foot(
                        self.leg_index, pos_base, valid_check=True), self.leg_index))
            self.robot_interface.pub_odom(torso_traj(t))
            rospy.sleep(0.01)


if __name__ == "__main__":

    # Optimize-based
    demo = TrajOptDemo()
    demo.viz_traj()
    rospy.sleep(1)
    demo.optimize_viz_rrt()
    # demo.optimize_viz()
    while rospy.is_shutdown() is False:
        demo.viz_robot_traj()
        rospy.sleep(1)
    rospy.spin()

    # RRT
    # demo = TrajOptDemo()
    # rospy.sleep(1)
    # demo.optimize_viz_rrt()
    # rospy.spin()
