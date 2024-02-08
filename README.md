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

![Co-simulation Framework](fast_legged_planner/doc/elspider_air_cosim_sch.png)
![Cosimulation example](fast_legged_planner/doc/eg_elspider_air_cosimulation.png)

Depend Repos:

- [Qrpucp/HexapodSoftware](https://github.com/Qrpucp/HexapodSoftware): check out branch `feature/co-simulation`

```bash
git clone --recursive git@github.com:Qrpucp/HexapodSoftware.git
git checkout feature/co-simulation
catkin_make
```

- [HITSME-HexLab/HexapodElevationMapping](https://github.com/HITSME-HexLab/HexapodElevationMapping): check out branch `feature/co-simulation` or `master`

```bash
git clone --recursive git@github.com:HITSME-HexLab/HexapodElevationMapping.git
git checkout feature/co-simulation
catkin build hexapod_elevation_mapping -DCMAKE_BUILD_TYPE=Release
```

Recommand workspace structure:

```txt
├── hexapod_ws
│   ├── build
│   ├── devel
│   └── src
│       └── HexapodSoftware
├── legged_ws
│   ├── build
│   ├── devel
│   ├── logs
│   └── src
│       ├── hpp-fcl
│       └── pinocchio
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
````

```bash
# Make sure depend repos are properly installed & sourced
catkin build fast_legged_planner -DCMAKE_BUILD_TYPE=Release
# Start HexapodSoftware Gazebo simulation
roslaunch user main.launch \
controller_type:=hlc \
robot_name:=elspider_air \
joystick_type:=keyboard_sim \
gazebo_hang_up:=on_ground \
interface_type:=gazebo
# New terminal, start Planner & elevation mapping
roslaunch fast_legged_planner elspider_air_simple_planner.launch
```

Settings are listed in `elspider_air_simple_planner.launch`.

## Documentations (A bit messy, temporarily not for new-comers)

Major rospkg:

- [fast_legged_planner](./fast_legged_planner/README.md)

Related rospkgs:

- [astar-algorithm-cpp](./astar-algorithm-cpp/README.md)
- [hexapod_robot_assets](./hexapod_robot_assets/README.md)

## Acknowledgements

- [astar-algorithm-cpp](https://github.com/justinhj/astar-algorithm-cpp)
- [Vertex Enumeration 3D](https://github.com/ZJU-FAST-Lab/VertexEnumeration3D)
