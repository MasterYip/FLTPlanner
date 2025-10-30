# Fast Legged Trajectory Planner

![banner](./doc/repo_banner.png)

<!-- NOTE/CAUTION/WARNING -->
> [!NOTE]
> This repo contains codes for paper [_KCFRC: Kinematic Collision-Aware Foothold Reachability Criteria for Legged Locomotion_](https://masteryip.github.io/fltplanner.github.io/).

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
ros-$ROS_DISTRO-octomap-msgs \
ros-$ROS_DISTRO-ompl \
ros-$ROS_DISTRO-serial \
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

### MCTS Contact Planner Examples

![FRC_EG](./doc/frc_examples.png)

Perform **foothold reachability checks** and trajectory optimization using recorded contact sequences or MCTS planner.

#### Hexapod Robot (ElSpider Air)

```bash
roslaunch legged_traj_plan_examples elspider_air_state_sequence_planner.launch \
robot_interface_type:=ElSpiderAirDummy \
sim:=true \
teleop_type:=keyboard \
demo_name:=4_ushape_barrier \
planner_cfg:=flt_cfg_planner_conv
```

Demos:
`1_stairs`, `2_stairs`, `3_quincuncial_piles`,
`4_barrier`, `4_ushape_barrier`, `5_channel`, `6_fractal`

Planners:
`flt_cfg_planner_keypoint`(KCFRC keypoint), `flt_cfg_planner_conv`(KCFRC conv),
`rrt_cfg_planner`, `stomp_cfg_planner`, `fec_planner`

More configs can be found in:

- [Launch Settings (Demos and Planners)](./legged_traj_planner/legged_traj_plan_examples/launch/elspider_air_state_sequence_planner.launch)
- [State Sequence Planner Configs](./legged_traj_planner/legged_traj_plan_examples/config/planner/state_sequence_planner.yaml)
- Swing Trajectory Planner Configs: In folder `./legged_traj_planner/legged_traj_plan/config/swing_traj_planner`

Robot Control:

- `arrow up`: Move forward
- `arrow down`: Move backward(not recommended)
- `arrow left`: Turn left(not recommended)
- `arrow right`: Turn right(not recommended)

> You can publish `geometry_msgs/Twist` to `/cmd_vel` to control the robot too.

#### Quadruped Robot (Unitree A1)

> Note: This demo are still under development, so the planner may not work properly.

```bash
roslaunch legged_traj_plan_examples unitree_a1_state_sequence_planner.launch \
robot_interface_type:=UnitreeA1Dummy \
sim:=true \
teleop_type:=keyboard \
demo_name:=6_fractal \
planner_cfg:=flt_cfg_planner_conv_a1
```

Demos:
`1_stairs`, `2_stairs`, `4_barrier`, `4_ushape_barrier`, `6_fractal`

Planners:
`flt_cfg_planner_keypoint_a1`(KCFRC keypoint), `flt_cfg_planner_conv_a1`(KCFRC conv),
`rrt_cfg_planner_a1`, `stomp_cfg_planner_a1`, `fec_planner_a1`

More configs can be found in:

- [Launch Settings (Demos and Planners)](./legged_traj_planner/legged_traj_plan_examples/launch/unitree_a1_state_sequence_planner.launch)
- [State Sequence Planner Configs](./legged_traj_planner/legged_traj_plan_examples/config/planner/unitree_a1_state_sequence_planner.yaml)
- Swing Trajectory Planner Configs: In folder `./legged_traj_planner/legged_traj_plan/config/swing_traj_planner`

Robot Control:

`keyboard`
- `W`: Move forward
- `S`: Move backward
- `A`: Move left
- `D`: Move right
- `Q`: Turn left
- `E`: Turn right

`PS5`
Joistick control the robot.

#### Hexapod Robot (Hexapod201)
![Hexapod201](./doc/hexapod201_foothold.png)
```bash
roslaunch legged_traj_plan_examples hexapod201_state_sequence_planner.launch \
robot_interface_type:=Hexapod201Dummy \
sim:=true \
teleop_type:=keyboard \
demo_name:=6_fractal \
planner_cfg:=height_clear_planner
```


### Raibert Heuristic Planner Examples

Perform trajectory optimization & reachability check using Raibert heuristic planner.

#### Hexapod Robot (ElSpider Air)

![Raibert_EG](./doc/raibert_elspider_air.png)

```bash
roslaunch legged_traj_plan_examples elspider_air_raibert_planner.launch \
robot_interface_type:=ElSpiderAirDummy \
sim:=true \
teleop_type:=PS5
```

```bash
roslaunch legged_traj_plan_examples elspider_air_simple_raibert_planner.launch \
robot_interface_type:=ElSpiderAirDummy \
sim:=true \
teleop_type:=PS5
```

#### Quadruped Robot (Unitree A1)

![Unitree A1](doc/raibert_unitree_a1.png)

```bash
roslaunch legged_traj_plan_examples unitree_a1_raibert_planner.launch \
robot_interface_type:=UnitreeA1Dummy \
sim:=true \
teleop_type:=PS5 \
demo_name:=6_fractal \
planner_cfg:=flt_cfg_planner_conv_a1
```

> Use Joystick to control the robot.

### Misc Examples

Examples for algorithm illustration.

<table>
  <tr>
    <td><img src="./legged_traj_planner/legged_traj_search_examples/doc/eg_gcs_rand_corridor_demo.png" alt="eg_gcs_rand_corridor_demo"></td>
    <td><img src="./legged_traj_planner/legged_traj_search_examples/doc/eg_gcs_rand_map_demo.png" alt="eg_gcs_rand_map_demo"></td>
  </tr>
</table>

Example List:

- eg_guide_surface_demo
- eg_convoluted_guide_surface_demo
- eg_keypoint_guide_surface_demo
- eg_gcs_barrier_demo
- eg_gcs_barrier_ani_demo
- eg_gcs_rand_corridor_demo
- eg_gcs_rand_map_demo

```bash
roslaunch legged_traj_search_examples gcs_example.launch \
example_name:=eg_gcs_barrier_ani_demo
```

### Auto Benchmarking

1. Update path in config file `legged_traj_planner/legged_traj_plan_examples/config/planner/state_sequence_planner_autobenchmark.yaml` to `analysis_scripts/benchmarking/temp`.

2. Run `analysis_scripts/benchmarking/ssplanner_auto_benchmark.py` to perform auto benchmarking of the state sequence planner. You can select the planner type and demo name in the script.

3. Run `analysis_scripts/benchmarking/auto_benchmark_analysis.ipynb` to perform analysis of the auto benchmarking results.

## Multi-Robot Support

The planner now supports both hexapod and quadruped robots through a unified interface:

### Supported Robot Types

- **Hexapod Robots**: 6-legged robots (e.g., ElSpider Air)
  - Uses bigait locomotion pattern
  - 6-leg reachability analysis
- **Quadruped Robots**: 4-legged robots (e.g., Unitree A1)
  - Uses trotting gait locomotion pattern
  - 4-leg reachability analysis
  - Enhanced velocity limits for dynamic locomotion

### Robot Detection

The `RaibertHeuristicPlanner` automatically detects the robot type by querying the robot interface and configures:

- Appropriate gait patterns (bigait for hexapod, trotting for quadruped)
- Correct number of leg schedulers and trajectories
- Robot-specific nominal foothold positions
- Leg-count appropriate reachability checks

### Usage

Both robot types use the same core planning algorithms but with automatically adapted parameters. Simply choose the appropriate launch file for your robot type, and the planner will handle the configuration.

## TODO

- [ ] Bugs exist in RobotInterface JacobianTimeVariation (test_robot_interface)

## Acknowledgements

This work is built upon the following open-source projects:

<!-- GCOPTER, STOMP, GridMap, OMPL etc -->

- [GCOPTER]
- [STOMP]
- [GridMap]
- [OMPL]
- [HPP-FCL]
- [Pinocchio]

<!-- Examples -->
<!-- <details>
  <summary><b>Setup env</b></summary>

  - Create a Conda env and setup the requirements:

```
conda create -n eg python=3.8
```
</details> -->
