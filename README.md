# fast_legged_planner test workspace

## Download

```bash
cd <planner_ws>
git clone --recursive git@github.com:MasterYip/Fast-Legged-Planner-Test.git
```

Then change the folder name to `src`.

## Important Examples

### ElSpider Air Co-simulation

A simple co-simulation for ElSpider Air.

<!-- ![Co-simulation Framework](legged_traj_plan_examples/doc/elspider_air_cosim_sch.png)
![Cosimulation example](legged_traj_plan_examples/doc/eg_elspider_air_cosimulation.png) -->

#### Dependent Repos

- [Qrpucp/HexapodSoftware](https://github.com/Qrpucp/HexapodSoftware): check out branch `feature/elspider_air_locomotion`

  ```bash
  git clone --recursive git@github.com:Qrpucp/HexapodSoftware.git
  git checkout feature/co-simulation
  catkin_make -DCMAKE_BUILD_TYPE=Release
  ```

  It needs pinocchio and hpp-fcl, you can install them by:

  ```bash
  # Clone pinocchio
  git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
  # Clone hpp-fcl
  git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
  catkin build pinocchio -DCMAKE_BUILD_TYPE=Release
  ```

- [HITSME-HexLab/HexapodElevationMapping](https://github.com/HITSME-HexLab/HexapodElevationMapping): check out branch `feature/co-simulation` or `master`

  ```bash
  git clone --recursive git@github.com:HITSME-HexLab/HexapodElevationMapping.git
  git checkout feature/co-simulation
  catkin build hexapod_elevation_mapping -DCMAKE_BUILD_TYPE=Release
  ```

#### Recommand workspace structure

```txt
├── hexapod_ws
│   ├── build
│   ├── devel
│   └── src
│       ├── hpp-fcl
│       ├── pinocchio
│       └── HexapodSoftware
├── perception_ws
│   ├── build
│   ├── devel
│   ├── logs
│   └── src
│       └── HexapodElevationMapping
└── planner_ws
    ├── build
    ├── devel
    ├── logs
    └── src
        ├── fast_legged_planner
        ├── hexapod_robot_assets
```

#### Get Started

##### Compile

```bash
# Make sure depend repos are properly installed & sourced
catkin build legged_traj_plan_examples -DCMAKE_BUILD_TYPE=Release
```

##### Launch procedure

FakeFeedback | Planning:

```bash
roslaunch legged_traj_plan_examples elspider_air_state_sequence_planner.launch \
robot_interface_type:=ElSpiderAirDummy \
sim:=true \
teleop_type:=keyboard
```

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

Legacy:

```bash
roslaunch legged_traj_plan_examples elspider_air_simple_raibert_vmc_planner.launch \
fake_feedback:=true \
sim:=true \
teleop_type:=PS5
```

Simulation | Perception & Planning & Control:

```bash
# Terminal1: start HexapodSoftware Gazebo simulation
roslaunch user main.launch \
controller_type:=hlc \
robot_name:=elspider_air \
joystick_type:=keyboard_sim \
gazebo_hang_up:=false \
interface_type:=gazebo
# Terminal2: start Planner & elevation mapping
roslaunch legged_traj_plan_examples elspider_air_state_sequence_planner.launch sim:=true
#or
roslaunch legged_traj_plan_examples elspider_air_raibert_vmc_planner.launch sim:=true
#or
roslaunch legged_traj_plan_examples elspider_air_simple_raibert_vmc_planner.launch sim:=true
```

Hardware | Perception & Planning & Control:

```bash
# Terminal1: start HexapodSoftware Hardware
# Get in to sudo mode
sudo su
source /home/user/HexLabCodeSpace/hexapod_ws/devel/setup.bash
# Start HexapodSoftware (after source the workspace)
roslaunch user main.launch \
controller_type:=hlc \
robot_name:=elspider_air \
joystick_type:=PS5 \
gazebo_hang_up:=false \
interface_type:=hardware
# Terminal2: start Planner & elevation mapping
roslaunch legged_traj_plan_examples elspider_air_state_sequence_planner.launch sim:=false use_optitrack:=true demo_name:=hardware
#or
roslaunch legged_traj_plan_examples elspider_air_raibert_planner.launch sim:=false use_optitrack:=true demo_name:=hardware
#or
roslaunch legged_traj_plan_examples elspider_air_raibert_vmc_planner.launch sim:=false
#or
roslaunch legged_traj_plan_examples elspider_air_simple_raibert_vmc_planner.launch sim:=false
```

Settings are listed in `elspider_air_simple_planner.launch`.

ROS bag recording:

Set the `rosbag_record:=true` in the launch file.

ROS bag playback:

```bash
roslaunch legged_traj_plan_examples rosbag_playback.launch
```

##### Usage

Robot initialization using keyboard (Terminal1):

- `o`: Motor initialization
- `u`: Press once to enter `sitdown mode`, press again to enter `standup mode`
- `v`: Enter `HLC mode` (communicate with planner bypassing topic `hexapod/foot_cmd_track`)

> For more details, please refer to HexapodSoftware `keyboard_sim` node.
> You can also use PS5 (sequence is R2, R1, A)

CmdVel control(Terminal2):

- `arrow up`: Move forward
- `arrow down`: Move backward(not recommended)
- `arrow left`: Turn left(not recommended)
- `arrow right`: Turn right(not recommended)

> You can publish `geometry_msgs/Twist` to `/cmd_vel` to control the robot too.

## Documentations (A bit messy, temporarily not for new-comers)

Major rospkg:

- [fast_legged_planner](./fast_legged_planner/README.md)

Related rospkgs:

- [astar-algorithm-cpp](./astar-algorithm-cpp/README.md)
- [hexapod_robot_assets](./hexapod_robot_assets/README.md)

## Acknowledgements

- [astar-algorithm-cpp](https://github.com/justinhj/astar-algorithm-cpp)
- [Vertex Enumeration 3D](https://github.com/ZJU-FAST-Lab/VertexEnumeration3D)
