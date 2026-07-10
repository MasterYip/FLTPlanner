# Project Hexapod201

## Installation

Clone the repository:

```bash
# Under catkin_ws
git clone --recursive https://github.com/MasterYip/FLTPlanner.git
mv FLTPlanner src
cd src
git submodule update --init --recursive
```

Install apt dependencies:

```bash
sudo apt install \
ros-$ROS_DISTRO-ros-industrial-cmake-boilerplate \
ros-$ROS_DISTRO-costmap-2d \
ros-$ROS_DISTRO-octomap \
ros-$ROS_DISTRO-ompl \
ros-$ROS_DISTRO-pcl-ros \
python3-catkin-tools \
qtbase5-dev \
libglpk-dev
```

Install cddlib manually:

```bash
# Under catkin_ws/src
cd ./legged_traj_planner/third_party
tar -xvf cddlib-0.94m.tar.gz
cd cddlib-0.94m
./configure
make
sudo make install
```

Build the package:

> [!WARNING]
> **DO NOT** install `ros-noetic-grid-map`, `ros-noetic-hpp-fcl` and `ros-noetic-pinocchio` from apt, which will lead to unexpected error.
> Recommand jobs of `catkin build -j16`
> | Jobs | Time Est. | Min Mem. |
> |------|----------|-----------------|
> | -j4 | 32min | 16 GB |
> | -j8 | 16min | 24 GB |
> | -j16 | 8min | 32 GB |
> | -j32 | 4min | 48 GB |

```bash
# Under catkin_ws
catkin build legged_traj_plan_examples legged_traj_search_examples robot_assets -DCMAKE_BUILD_TYPE=RelWithDebInfo # Release
source ./devel/setup.bash
```

### Problem Shooting

1. Undefined reference to 'grid_map::GridMap::add()'
    - If you encounter this error, it may be due to the `grid_map` library is linked incorrectly. Make sure you have already uninstall the ros version of `grid_map`: `sudo apt remove ros-noetic-grid-map-*`


## Get Started

Install pyads for PLC communication:

```bash
pip install pyads
```

### Python interface (Base motion & Foothold):

![alt text](doc/hexapod201_basenav.png)

```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201Dummy \
sim:=true \
teleop_type:=keyboard \
demo_name:=6_fractal \
planner_cfg:=height_clear_planner \
use_pyinterface:=true
```

**New ROS interface**:
1. Simulation with fake map
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201ROS \
sim:=true \
teleop_type:=keyboard \
demo_name:=7_prj201_nav \
planner_cfg:=height_clear_planner \
use_pyinterface:=true
```

2. Simulation with fake map with real command
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201ROS \
sim:=true \
teleop_type:=keyboard \
interface_type:=dummy_with_real_cmd \
demo_name:=7_prj201_barrier \
planner_cfg:=height_clear_planner \
use_pyinterface:=true
```

3. Real robot with real map (DEPRECATED)
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201ROS \
sim:=false \
teleop_type:=keyboard \
demo_name:=7_prj201 \
planner_cfg:=height_clear_planner \
use_pyinterface:=true
```

### Sequential Long-Distance Navigation (Simulation with fake map)

This demo tests the sequential navigation behavior: give a pose2d goal (possibly **outside** the known fake map), and watch the robot pick safe sub-destinations near the traversable area boundary and navigate step by step.

The `7_prj201_nav` demo provides a 16m × 6m fake map with obstacles and a barrier. The traversability layer is computed from the static terrain image, so "outside the map" means any cell whose height is unknown (NaN) — the robot will find the farthest traversable point and walk there.

```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
  robot_interface_type:=Hexapod201ROS \
  sim:=true \
  teleop_type:=keyboard \
  demo_name:=7_prj201_nav \
  planner_cfg:=height_clear_planner \
  use_pyinterface:=true
```

**In RViz**:

1. Click **`2D Pose Estimate`** (not `2D Nav Goal`) to set a target point.
2. If the target is within the traversable map → the robot plans an RRT path and walks directly there.
3. If the target is **outside** the traversable map (beyond the map edge or inside an obstacle) → the planner:
   - Calls `findSafeSubDestination()` to find the farthest traversable point along the direction to the goal
   - Plans an RRT path to that sub-destination
   - Walks there using the tripod gait
   - Since `sim:=true`, the fake map does **not** update, so the next iteration will again find a sub-destination at the same boundary. On hardware the map would rebuild from fresh stationary LiDAR scans.

**What to observe**:
- The sub-destination should be near the map/terrain boundary, on **flat safe ground** (traversable cells with no NaN).
- All six feet stay within the traversable region — no footstep planned on unknown/obstacle cells.
- The robot stops briefly at each sub-destination, then continues toward the goal.

### Cpp Dummy interface (Base motion & Foothold):

![alt text](doc/hexapod201_foothold.png)

Joypad with Swing Trajactory:
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201Dummy \
sim:=true \
teleop_type:=PS5 \
demo_name:=6_fractal \
planner_cfg:=height_clear_planner \
use_pyinterface:=false
```

Keyboard with only pose planning:
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201Dummy \
sim:=true \
teleop_type:=keyboard \
demo_name:=6_fractal \
planner_cfg:=height_clear_planner \
use_pyinterface:=false
```

## Prompt

#file:DummyHexapod201InterfaceROS.h  is a little bit messy: when config_.usePyInterface=True, it communicate with #file:hexapod201_interface.py , when it is False, it set the state directly from command.
I hope to implement a new #file:Hexapod201InterfaceROS.h . This interface read states only from #file:hexapod201_interface.py , and set command to #file:hexapod201_interface.py . So for #file:Hexapod201InterfaceROS.h , #file:hexapod201_interface.py  is the "real robot". The communication is done through rostopic. This makes the program more plain and clean.
As for #file:hexapod201_interface.py , we have a dummy class and a real robot class. The dummy one holds robot states, i.e. once command is received, it update its state according to the command. The real one is able to read real robot state and set real command.
What I need you to do:
1. Write a new #file:Hexapod201InterfaceROS.h .
2. update #file:hexapod201_interface.py to make the interface well defined.
3. Ensure rostopic communication between #file:hexapod201_interface.py  and #file:Hexapod201InterfaceROS.h .