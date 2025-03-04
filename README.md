# Fast Legged Trajectory Planner
<!-- NOTE/CAUTION/WARNING -->
> [!NOTE]
> This repo contains codes for paper `KCFRC: Kinematic Collision-Aware Foothold Reachability Criteria for Legged Locomotion`.

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

```bash
# Under catkin_ws/src
catkin build legged_traj_plan_examples legged_traj_search_examples hexapod_robot_assets -DCMAKE_BUILD_TYPE=Release
source ../devel/setup.bash
```

## Get Started

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
