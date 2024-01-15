# fast_legged_planner

Test repo for Fast-Legged-Planner

## Setup

### Robotic Course Setup

- `Meshcat` is used for visualization.

- `pinocchio` is used for kinematics and dynamics.

Create and activate conda environment:

```bash
conda env create -f robotic_course_env.yml
```

### Deps

- pinocchio(with python binding)

  IMPORTANT: This is conflict with legged_control
  In ~/.bashrc:

```bash
# Pinocchio (IMPORTANT: This may interfere with legged_ws-pinocchio)
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

- grid_map
- ompl(with python binding)

## Examples

### Elspider2 walking with MCTs planner

A complete test for swing trajectory optimization.

![Elspider2 walking with MCTs planner](doc/eg_hitspider_walking.png)

```bash
source .setup_rc_nuc11 # setup env (WITH_PINPY = 1)
roslaunch fast_legged_planner HIT_spider_traj_planner.launch
# after rviz is loaded, cd MCTs project(MPI_CPP_Version, branch feature/swing_leg_planner/test)
./bin/parallelMCTS_VirtualLoss # start MCTs planner
```

Several settings is provided in `HIT_spider_traj_planner.launch`, planner settings is in python codes(temporarily).

NOTE: hexapod_State.msg has been updated (line 17 and below is newly added), but `parallelMCTS_VirtualLoss` used a old version of hexapod_State.msg, so you need to modify it manually.

```cpp

### Trajectory Optimization Demo

A simple trajectory optimization demo for various planning algorithms(RRT, BFGS, etc.).

 ![Trajectory Optimization Demo](doc/eg_traj_opt_demo.png)

```bash
source .setup_rc_nuc11 # setup env (WITH_PINPY = 1)
roslaunch fast_legged_planner traj_opt_demo.launch
```

Several settings is provided in `traj_opt_demo.launch`, planner settings is in python codes(temporarily).

### Purposed(cvxhull) Trajectory Optimization Demo

A simple trajectory optimization demo for GCS-based trajectory optimization.

![eg_gcs_traj_opt](doc/eg_gcs_traj_opt.png)

Coming soon.

## Note

- chmod python scripts under /scripts in order to run it

## Interface Explanation

### MCT planner

Needs grid_map with:

- elevation
- normal_x
- normal_y
- normal_z

Messages difinition:

- [FeetPosition.msg](./msg/FeetPosition.msg)
- [hexapod_Base_Pose.msg](./msg/hexapod_Base_Pose.msg)
- [hexapod_RPY.msg](./msg/hexapod_RPY.msg)
- [hexapod_State.msg](./msg/hexapod_State.msg)

### HexapodSoftware

HLC (High Level Controller)

Msg difinition:

- [FootCmd.msg](./msg/FootCmd.msg)
