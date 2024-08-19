#!/usr/bin/env python
# coding=utf-8
'''
Author: HexLab-NUC12-MasterYip 2205929492@qq.com
Date: 2024-08-18 21:29:09
Description: file content
FilePath: /planner_ws/src/analysis_scripts/benchmarking/ssplanner_auto_benchmark.py
LastEditTime: 2024-08-19 11:41:20
LastEditors: HexLab-NUC12-MasterYip
'''

import os
import yaml
import roslaunch
import rospy
import std_msgs.msg as msg
import pandas as pd
import numpy as np


# Directory Management
try:
    # Run in Terminal
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    # Run in ipykernel & interactive
    ROOT_DIR = os.getcwd()


def csv2dict(filename):
    # ignore spaces
    df = pd.read_csv(filename, sep=",", skipinitialspace=True)
    return df.to_dict(orient="list")


class TestCase:
    def __init__(self, planner_name, demo_name,
                 with_ceiling=False, rosbag_record=False):
        self.planner_name = planner_name
        self.demo_name = demo_name
        self.with_ceiling = with_ceiling
        self.rosbag_record = rosbag_record

        # Defaults
        self.sim = True
        self.fake_feedback = True
        self.teleop_type = "keyboard"
        self.rviz_gui = False
        self.output = "log"  # screen, log

        # Benchmark
        self.tot_time = 0
        self.suc_rate = 0
        self.smoothness = 0
        self.avg_time = 0
        self.max_time = 0
        self.min_time = 0
        self.std_time = 0

    @property
    def to_dict(self):
        return {"planner_cfg": self.planner_name,
                "demo_name": self.demo_name,
                "with_ceiling": "true" if self.with_ceiling else "false",
                "rosbag_record": "true" if self.rosbag_record else "false",
                "sim": "true" if self.sim else "false",
                "fake_feedback": "true" if self.fake_feedback else "false",
                "teleop_type": self.teleop_type,
                "rviz_gui": "true" if self.rviz_gui else "false",
                "output": self.output}

    def parse_planner_benchmark(self, planner_benchmark: dict):
        self.tot_time = planner_benchmark["Totaltime"]

    def parse_swingtraj_benchmark(self, swingtraj_benchmark: dict):
        opttime_list = swingtraj_benchmark["totTime"]
        success_list = swingtraj_benchmark["optRetType"]
        self.suc_rate = np.sum(success_list) / len(success_list)
        self.avg_time = np.mean(opttime_list)
        self.max_time = np.max(opttime_list)
        self.min_time = np.min(opttime_list)
        self.std_time = np.std(opttime_list)

    def print_benchmark(self):
        print("=====================================")
        print(f"Planner: {self.planner_name}, Demo: {self.demo_name}")
        print(f"Total Time: {self.tot_time}")
        print(f"Success Rate: {self.suc_rate}")
        print(f"Average Time: {self.avg_time}")
        print(f"Max Time: {self.max_time}")
        print(f"Min Time: {self.min_time}")
        print(f"Std Time: {self.std_time}")


class SSPlannerAutoBenchmark:
    pkg_name = "legged_traj_plan_examples"
    launch_file = "elspider_air_state_sequence_planner.launch"

    # Benchmark
    planner_benchmark = "StateSequencePlannerBenchmark.yaml"
    swingtraj_benchmark = "SwingTrajPlannerBenchmark.csv"
    robot_profile = "RobotProfileRecord.csv"

    def __init__(self):
        rospy.init_node("ssplanner_auto_benchmark", anonymous=True)
        self.progress_sub = rospy.Subscriber("/benchmark_progress",
                                             msg.Bool,
                                             self.progress_callback)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.is_done = False
        self.test_cases = []
        self.test_case_ptr = 0
        pass

    def run(self, testcase: TestCase, timeout=None):
        """Run single test

        Args:
            testcase (TestCase): _description_
            timeout (_type_, optional): _description_. Defaults to None.
        """
        cli_args = [self.pkg_name, self.launch_file]
        for key, value in testcase.to_dict.items():
            cli_args.append(key+":="+value)
        print(cli_args)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        launch_files = [(launch_file, cli_args)]
        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, launch_files)
        self.parent.start()
        if (timeout):
            rospy.sleep(timeout)
            self.parent.shutdown()

    def run_tests(self, testcases, timeout=None):
        """Batch run tests

        Args:
            testcases (_type_): _description_
            timeout (_type_, optional): _description_. Defaults to None.
        """
        self.is_done = False
        self.test_cases = testcases
        self.test_case_ptr = 0
        self.run(self.test_cases[self.test_case_ptr], timeout)

    def progress_callback(self, msg):
        self.parent.shutdown()
        self.analyze()
        self.test_case_ptr += 1
        if self.test_case_ptr < len(self.test_cases):
            self.run(self.test_cases[self.test_case_ptr])
        else:
            self.summary()
            rospy.signal_shutdown("Benchmark finished.")

    def get_abs_path(self, filename):
        return os.path.join(ROOT_DIR, "temp", filename)

    def analyze(self):
        with open(self.get_abs_path(self.planner_benchmark), "r") as f:
            planner_benchmark = yaml.load(f, Loader=yaml.FullLoader)
            self.test_cases[self.test_case_ptr].parse_planner_benchmark(planner_benchmark)
        with open(self.get_abs_path(self.swingtraj_benchmark), "r") as f:
            swingtraj_benchmark = csv2dict(f)
            self.test_cases[self.test_case_ptr].parse_swingtraj_benchmark(swingtraj_benchmark)
        with open(self.get_abs_path(self.robot_profile), "r") as f:
            robot_profile = csv2dict(f)
        pass

    def summary(self):
        for testcase in self.test_cases:
            testcase.print_benchmark()
        pass


if __name__ == "__main__":
    benchmark = SSPlannerAutoBenchmark()
    testcases = []
    testcases.append(TestCase("flt_cfg_planner", "4_barrier", False, False))
    testcases.append(TestCase("flt_cfg_planner", "4_ushape_barrier", False, False))
    testcases.append(TestCase("rrt_cfg_planner", "2_stairs", False, False))
    benchmark.run_tests(testcases)
    rospy.spin()
