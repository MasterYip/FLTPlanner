import sys
if "." not in sys.path:
    sys.path.append(".")
from fast_legged_planner_py.robot_interface.ur5_robotinterface import UR5_RobotInterface

ur5 = UR5_RobotInterface()
ur5.print_joints()
