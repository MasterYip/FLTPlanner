#!/usr/bin/env python
# coding=utf-8
'''
Author: HexLab-NUC12-MasterYip 2205929492@qq.com
Date: 2024-08-18 21:29:09
Description: file content
FilePath: /planner_ws/src/analysis_scripts/benchmarking/ssplanner_auto_benchmark.py
LastEditTime: 2025-02-21 14:42:22
LastEditors: HexLab-NUC12-MasterYip
'''

from typing import Tuple, List
import os
import yaml
import roslaunch
import rospy
import std_msgs.msg as msg
import pandas as pd
import numpy as np
import json
import datetime

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


# A1-specific planners for quadruped locomotion
A1_PLANNERS = [
    "flt_cfg_groundtruth_a1",
    "flt_cfg_planner_conv_a1",
    "flt_cfg_planner_keypoint_a1",
    "rrt_cfg_planner_a1", 
    "stomp_cfg_planner_a1",
    "fec_planner_a1",
]

# A1-specific demos (simpler terrain for quadruped testing)
A1_DEMOS = [
    ("2_stairs", False),
    ("6_fractal", False),
    # ("4_barrier", True),
    # ("5_channel", True),
]

PLANNERS = [
    "rrt_cfg_groundtruth",  # As the ground truth for Reachable Evaluation
    "flt_cfg_groundtruth", 
    "flt_cfg_planner_keypoint", 
    "flt_cfg_planner_conv", 
    # "minco_cfg_planner",
    "rrt_cfg_planner",
    "stomp_cfg_planner",
    # "stomp_cfg_groundtruth", # As the ground truth for Reachable Evaluation
    "fec_planner",
    # "height_clear_planner",
    # "rrt_planner",
]

DEMOS = [
    ("2_stairs", False),
    ("3_quincuncial_piles", False),
    ("4_ushape_barrier", True),
    ("4_barrier", True),
    ("4_barrier_vague", True),
    ("5_channel", True),
    ("6_fractal", False),
]


class TestCase:
    def __init__(self, planner_name, demo_name,
                 with_ceiling=False, rosbag_record=False, robot_type="elspider"):
        self.planner_name = planner_name
        self.demo_name = demo_name
        self.with_ceiling = with_ceiling
        self.rosbag_record = rosbag_record
        self.robot_type = robot_type  # "elspider" or "a1"

        # Default Parameters - vary by robot type
        self.sim = True
        if robot_type == "a1":
            self.robot_interface_type = "UnitreeA1Dummy"
        else:
            self.robot_interface_type = "ElSpiderAirDummy"
        self.teleop_type = "keyboard"
        self.rviz_gui = False
        self.auto_benchmark = True
        self.output = "log"  # screen, log

        # SwingTraj Optimization Benchmark
        self.opt_num = 0
        self.suc_num = 0
        self.suc_rate = 0
        self.tot_time = 0
        # Statistics per trajectory
        self.ave_time = 0
        self.ave_time2 = 0
        self.max_time = 0
        self.min_time = 0
        self.std_time = 0

        self.ave_len = 0
        self.ave_len2 = 0
        self.max_len = 0
        self.min_len = 0
        self.std_len = 0

        self.ave_ctrl = 0
        self.ave_ctrl2 = 0
        self.max_ctrl = 0
        self.min_ctrl = 0
        self.std_ctrl = 0

        # Reachability Check Benchmark
        self.rc_totchecknum = 0  # points
        self.rc_reachablenum = 0  # points
        self.rc_totlegchecknum = 0  # legs
        self.rc_tottime = 0      # milliseconds
        # Statistics per leg
        self.rc_avetime = 0     # per leg
        self.rc_avetime2 = 0
        self.rc_maxtime = 0
        self.rc_mintime = 0
        self.rc_stdtime = 0

        # Reachable Array
        self.reachable_array = []

    @property
    def rl_args(self):
        """roslaunch arguments"""
        return {"planner_cfg": self.planner_name,
                "demo_name": self.demo_name,
                "with_ceiling": "true" if self.with_ceiling else "false",
                "rosbag_record": "true" if self.rosbag_record else "false",
                "sim": "true" if self.sim else "false",
                "robot_interface_type": self.robot_interface_type,
                "teleop_type": self.teleop_type,
                "rviz_gui": "true" if self.rviz_gui else "false",
                "auto_benchmark": "true" if self.auto_benchmark else "false",
                "output": self.output}

    @property
    def benchmark_dict(self):
        return {
            # Trajectory Optimization Benchmark
            "OptNum": self.opt_num,
            "SuccessNum": self.suc_num,
            "SuccessRate": self.suc_rate,
            "Totaltime": self.tot_time,
            # Times
            "AveTime": self.ave_time,
            "AveTime2": self.ave_time2,
            "MaxTime": self.max_time,
            "MinTime": self.min_time,
            "StdTime": self.std_time,
            # Lengths
            "AveLen": self.ave_len,
            "AveLen2": self.ave_len2,
            "MaxLen": self.max_len,
            "MinLen": self.min_len,
            "StdLen": self.std_len,
            # Controls
            "AveCtrl": self.ave_ctrl,
            "AveCtrl2": self.ave_ctrl2,
            "MaxCtrl": self.max_ctrl,
            "MinCtrl": self.min_ctrl,
            "StdCtrl": self.std_ctrl,
            # Reachability Check Benchmark
            "RcTotCheckNum": self.rc_totchecknum,
            "RcReachableNum": self.rc_reachablenum,
            "RcTotLegCheckNum": self.rc_totlegchecknum,
            "RcTotTime": self.rc_tottime,
            # Times (per leg)
            "RcAveTime": self.rc_avetime,
            "RcAveTime2": self.rc_avetime2,
            "RcMaxTime": self.rc_maxtime,
            "RcMinTime": self.rc_mintime,
            "RcStdTime": self.rc_stdtime,
        }

    def parse_planner_benchmark(self, planner_benchmark: dict):
        # self.tot_time = planner_benchmark["Totaltime"]
        # NOTE: this will include benchmark data calculation time, do not use
        pass

    def parse_swingtraj_benchmark(self, swingtraj_benchmark: dict):
        opttime_list = swingtraj_benchmark["totTime"]
        success_list = swingtraj_benchmark["optRetType"]
        trajlen_list = swingtraj_benchmark["trajLen"]
        # NOTE: convert to float to avoid JSON serialization error
        trajlen_list = [float(x) for x in trajlen_list]
        trajlen_succ_only = np.array(trajlen_list)[np.array(success_list) == 1]
        trajctrl_list = swingtraj_benchmark["trajCtrl"]
        # NOTE: convert to float to avoid JSON serialization error
        trajctrl_list = [float(x) for x in trajctrl_list]
        trajctrl_succ_only = np.array(trajctrl_list)[np.array(success_list) == 1]

        self.opt_num = len(opttime_list)
        self.suc_num = int(np.sum(success_list))
        self.suc_rate = np.sum(success_list) / len(success_list)
        self.tot_time = np.sum(opttime_list)

        # Times (include failed cases)
        self.ave_time = np.mean(opttime_list)
        self.ave_time2 = np.mean(np.array(opttime_list) ** 2)  # to calculate total std
        self.max_time = np.max(opttime_list)
        self.min_time = np.min(opttime_list)
        self.std_time = np.std(opttime_list)

        # Lengths (only successful cases)
        self.ave_len = np.mean(trajlen_succ_only)
        self.ave_len2 = np.mean(trajlen_succ_only ** 2)
        self.max_len = np.max(trajlen_succ_only)
        self.min_len = np.min(trajlen_succ_only)
        self.std_len = np.std(trajlen_succ_only)

        # Controls (only successful cases)
        self.ave_ctrl = np.mean(trajctrl_succ_only)
        self.ave_ctrl2 = np.mean(trajctrl_succ_only ** 2)
        self.max_ctrl = np.max(trajctrl_succ_only)
        self.min_ctrl = np.min(trajctrl_succ_only)
        self.std_ctrl = np.std(trajctrl_succ_only)

    def parse_reachable_benchmark(self, reachable_benchmark: dict):
        tottime_list = reachable_benchmark["totTime"]
        totnum_list = reachable_benchmark["total"]
        reachablenum_list = reachable_benchmark["reachable"]
        legindex_list = reachable_benchmark["index"]
        # NOTE: using int() to avoid JSON serialization error
        self.rc_totchecknum = int(np.sum(totnum_list))
        self.rc_reachablenum = int(np.sum(reachablenum_list))
        self.rc_totlegchecknum = int(len(legindex_list))
        self.rc_tottime = np.sum(tottime_list)

        self.rc_avetime = np.mean(tottime_list)
        self.rc_avetime2 = np.mean(np.array(tottime_list) ** 2)
        self.rc_maxtime = np.max(tottime_list)
        self.rc_mintime = np.min(tottime_list)
        self.rc_stdtime = np.std(tottime_list)

        for i in range(totnum_list[0]):
            self.reachable_array.append(reachable_benchmark[f"r{i}"])
        self.reachable_array = np.array(self.reachable_array).transpose().tolist()

    def print_benchmark(self):
        print("=====================================")
        print(f"Planner: {self.planner_name}, Demo: {self.demo_name}")
        print(f"Total Time: {self.tot_time}")
        print(f"Average Time: {self.ave_time}")
        print(f"Max Time: {self.max_time}")
        print(f"Min Time: {self.min_time}")
        print(f"Std Time: {self.std_time}")
        print(f"Success Rate: {self.suc_rate}")
        print(f"Average Length: {self.ave_len}")
        print(f"Average Control: {self.ave_ctrl}")


class SSPlannerAutoBenchmark:
    pkg_name = "legged_traj_plan_examples"
    launch_file = "elspider_air_state_sequence_planner.launch"

    # Benchmark
    planner_benchmark = "StateSequencePlannerBenchmark.yaml"
    swingtraj_benchmark = "OptBenchmark.csv"
    reachable_benchmark = "ReachableBenchmark.csv"
    robot_profile = "RobotProfileRecord.csv"

    def __init__(self):
        rospy.init_node("ssplanner_auto_benchmark", anonymous=True)
        self.progress_sub = rospy.Subscriber("/benchmark_progress",
                                             msg.Bool,
                                             self.progress_callback)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.test_cases = []
        self.test_case_ptr = 0
        pass

    def run(self, testcase: TestCase, timeout=None):
        """run single test"""
        cli_args = [self.pkg_name, self.launch_file]
        for key, value in testcase.rl_args.items():
            cli_args.append(key+":="+value)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        launch_files = [(launch_file, cli_args)]
        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, launch_files)
        self.parent.start()
        if (timeout):
            rospy.sleep(timeout)
            self.parent.shutdown()

    def run_tests(self, testcases, timeout=None):
        """run tests list"""
        self.test_cases = testcases
        self.test_case_ptr = 0
        self.run(self.test_cases[self.test_case_ptr], timeout)

    def run_benchmark(self, planners=PLANNERS, demos: List[Tuple[str, bool]] = DEMOS):
        self.planners = planners
        self.demos = demos
        self.test_cases = []
        # Generate test cases
        for planner in planners:
            for demo in demos:
                self.test_cases.append(TestCase(planner, demo[0], demo[1], False))
        self.test_case_ptr = 0
        self.run(self.test_cases[self.test_case_ptr])

    def save_benchmark(self, filename="AutoBenchmarkOutput", timestamp=True):
        benchmark = {}
        self.test_case_ptr = 0
        for planner in self.planners:
            benchmark[planner] = {}
            for demo in self.demos:
                benchmark[planner][demo[0]] = self.test_cases[self.test_case_ptr].benchmark_dict
                self.test_case_ptr += 1
        if timestamp:
            filename = filename + "_" + datetime.datetime.now().strftime("%Y%m%d") + ".json"
        else:
            filename = filename + ".json"
        abspath = os.path.join(ROOT_DIR, "data", filename)
        with open(abspath, "w") as f:
            json.dump(benchmark, f, indent=4)
        return

    def save_reachable_array(self, filename="AutoBenchmarkReachableArray", timestamp=True):
        benchmark = {}
        self.test_case_ptr = 0
        for planner in self.planners:
            benchmark[planner] = {}
            for demo in self.demos:
                benchmark[planner][demo[0]] = self.test_cases[self.test_case_ptr].reachable_array
                self.test_case_ptr += 1
        if timestamp:
            filename = filename + "_" + datetime.datetime.now().strftime("%Y%m%d") + ".json"
        else:
            filename = filename + ".json"
        abspath = os.path.join(ROOT_DIR, "data", filename)
        with open(abspath, "w") as f:
            json.dump(benchmark, f, indent=4)
        return

    def progress_callback(self, msg):
        self.parent.shutdown()
        self.analyze()
        self.test_case_ptr += 1
        rospy.sleep(0.5)
        if self.test_case_ptr < len(self.test_cases):
            self.run(self.test_cases[self.test_case_ptr])
        else:
            rospy.signal_shutdown("Benchmark finished.")

    def get_abs_path(self, filename):
        return os.path.join(ROOT_DIR, "temp", filename)

    def analyze(self):
        if not os.path.exists(os.path.join(ROOT_DIR, "temp")):
            os.makedirs(os.path.join(ROOT_DIR, "temp"))
        with open(self.get_abs_path(self.planner_benchmark), "r") as f:
            planner_benchmark = yaml.load(f, Loader=yaml.FullLoader)
            self.test_cases[self.test_case_ptr].parse_planner_benchmark(planner_benchmark)
        with open(self.get_abs_path(self.swingtraj_benchmark), "r") as f:
            swingtraj_benchmark = csv2dict(f)
            self.test_cases[self.test_case_ptr].parse_swingtraj_benchmark(swingtraj_benchmark)
        with open(self.get_abs_path(self.reachable_benchmark), "r") as f:
            reachable_benchmark = csv2dict(f)
            self.test_cases[self.test_case_ptr].parse_reachable_benchmark(reachable_benchmark)
        with open(self.get_abs_path(self.robot_profile), "r") as f:
            robot_profile = csv2dict(f)
        pass

    def summary(self):
        for testcase in self.test_cases:
            testcase.print_benchmark()
        pass


class A1StateSequencePlannerAutoBenchmark(SSPlannerAutoBenchmark):
    """A1-specific benchmark class that inherits from the base class"""
    
    # A1-specific benchmark file names
    planner_benchmark = "StateSequencePlannerBenchmark.yaml"
    swingtraj_benchmark = "OptBenchmark.csv"
    reachable_benchmark = "ReachableBenchmark.csv"
    robot_profile = "RobotProfileRecord.csv"

    def __init__(self):
        super().__init__()
        # Override launch file for A1
        self.launch_file = "unitree_a1_state_sequence_planner.launch"
        

    def run_benchmark(self, planners=A1_PLANNERS, demos: List[Tuple[str, bool]] = A1_DEMOS):
        """Run A1-specific benchmark with default A1 planners and demos"""
        self.planners = planners
        self.demos = demos
        self.test_cases = []
        # Generate test cases for A1
        for planner in planners:
            for demo in demos:
                self.test_cases.append(TestCase(planner, demo[0], demo[1], False, robot_type="a1"))
        self.test_case_ptr = 0
        self.run(self.test_cases[self.test_case_ptr])

    def save_benchmark(self, filename="A1AutoBenchmarkOutput", timestamp=True):
        """Save A1 benchmark results with A1-specific filename"""
        benchmark = {}
        self.test_case_ptr = 0
        for planner in self.planners:
            benchmark[planner] = {}
            for demo in self.demos:
                benchmark[planner][demo[0]] = self.test_cases[self.test_case_ptr].benchmark_dict
                self.test_case_ptr += 1
        if timestamp:
            filename = filename + "_" + datetime.datetime.now().strftime("%Y%m%d") + ".json"
        else:
            filename = filename + ".json"
        abspath = os.path.join(ROOT_DIR, "data", filename)
        with open(abspath, "w") as f:
            json.dump(benchmark, f, indent=4)
        return

    def save_reachable_array(self, filename="A1AutoBenchmarkReachableArray", timestamp=True):
        """Save A1 reachable array with A1-specific filename"""
        benchmark = {}
        self.test_case_ptr = 0
        for planner in self.planners:
            benchmark[planner] = {}
            for demo in self.demos:
                benchmark[planner][demo[0]] = self.test_cases[self.test_case_ptr].reachable_array
                self.test_case_ptr += 1
        if timestamp:
            filename = filename + "_" + datetime.datetime.now().strftime("%Y%m%d") + ".json"
        else:
            filename = filename + ".json"
        abspath = os.path.join(ROOT_DIR, "data", filename)
        with open(abspath, "w") as f:
            json.dump(benchmark, f, indent=4)
        return


def run_a1_benchmark():
    """Run A1-specific benchmark"""
    benchmark = A1StateSequencePlannerAutoBenchmark()
    benchmark.run_benchmark(A1_PLANNERS, A1_DEMOS)
    rospy.spin()
    benchmark.save_benchmark()
    benchmark.save_reachable_array()


def run_a1_tests():
    """Run A1-specific tests"""
    benchmark = A1StateSequencePlannerAutoBenchmark()
    testcases = []
    testcases.append(TestCase("raibert_heuristic_planner", "2_stairs", False, False, robot_type="a1"))
    testcases.append(TestCase("flt_cfg_planner", "4_barrier", True, False, robot_type="a1"))
    testcases.append(TestCase("rrt_cfg_planner", "5_channel", True, False, robot_type="a1"))
    testcases.append(TestCase("stomp_cfg_planner", "3_quincuncial_piles", False, False, robot_type="a1"))
    benchmark.run_tests(testcases)
    rospy.spin()
    benchmark.summary()


def run_benchmark():
    benchmark = SSPlannerAutoBenchmark()
    benchmark.run_benchmark(PLANNERS, DEMOS)
    rospy.spin()
    benchmark.save_benchmark()
    benchmark.save_reachable_array()


def run_tests():
    benchmark = SSPlannerAutoBenchmark()
    testcases = []
    testcases.append(TestCase("flt_cfg_planner", "4_ushape_barrier", True, False))
    testcases.append(TestCase("flt_cfg_planner_fast", "4_ushape_barrier", True, False))
    testcases.append(TestCase("rrt_cfg_planner", "4_ushape_barrier", True, False))
    testcases.append(TestCase("stomp_cfg_planner", "4_ushape_barrier", True, False))
    # testcases.append(TestCase("flt_cfg_planner", "4_ushape_barrier", True, False))
    # testcases.append(TestCase("rrt_cfg_planner", "2_stairs", False, False))
    benchmark.run_tests(testcases)
    rospy.spin()
    benchmark.summary()


if __name__ == "__main__":
    # Uncomment the desired benchmark to run:
    
    # Run ElSpider Air (hexapod) benchmark (original functionality)
    # run_benchmark()
    
    # Run A1 (quadruped) benchmark
    run_a1_benchmark()
    
    # Run ElSpider Air tests
    # run_tests()
    
    # Run A1 tests
    # run_a1_tests()
    pass
