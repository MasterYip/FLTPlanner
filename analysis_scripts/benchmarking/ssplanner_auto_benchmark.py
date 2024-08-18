#!/usr/bin/env python
# coding=utf-8
'''
Author: HexLab-NUC12-MasterYip 2205929492@qq.com
Date: 2024-08-18 21:29:09
Description: file content
FilePath: /planner_ws/src/analysis_scripts/benchmarking/ssplanner_auto_benchmark.py
LastEditTime: 2024-08-18 22:47:51
LastEditors: HexLab-NUC12-MasterYip
'''

from operator import truediv
import yaml
import roslaunch
import rospy
import std_msgs.msg as msg

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
    
    @property
    def to_dict(self):
        return {"planner_name": self.planner_name, 
                "demo_name": self.demo_name, 
                "with_ceiling": "true" if self.with_ceiling else "false", 
                "rosbag_record": "true" if self.rosbag_record else "false", 
                "sim": "true" if self.sim else "false", 
                "fake_feedback": "true" if self.fake_feedback else "false", 
                "teleop_type": self.teleop_type}
    
    
        
    

class SSPlannerAutoBenchmark:
    pkg_name = "legged_traj_plan_examples"
    launch_file = "elspider_air_state_sequence_planner.launch"
    # launch_file = "key_teleop.launch"
    def __init__(self):
        rospy.init_node("ssplanner_auto_benchmark", anonymous=True)
        self.progress_sub = rospy.Subscriber("/benchmark_progress", 
                                             msg.Bool, 
                                             self.progress_callback)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # self.launch = roslaunch.scriptapi.ROSLaunch()
        # self.launch.start()
        pass

    def run(self, testcase: TestCase, timeout=None):
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
        else:
            rospy.spin()
        
    def progress_callback(self, msg):
        self.parent.shutdown()

if __name__ == "__main__":
    benchmark = SSPlannerAutoBenchmark()
    # benchmark.run({"planner_name": "flt_cfg_planner", 
    #                 "demo_name": "2_stairs", 
    #                 "with_ceiling": "false", 
    #                 "rosbag_record": "false"})
    testcase0 = TestCase("flt_cfg_planner", "2_stairs", False, False)
    benchmark.run(testcase0)
    


