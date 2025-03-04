# KCFRC

## Dependencies

```bash
sudo apt install \
ros-$ROS_DISTRO-ros-industrial-cmake-boilerplate \
ros-$ROS_DISTRO-costmap-2d \
libglpk-dev
```

Install cddlib manually:

```bash
cd src/legged_traj_planner/third_party/cddlib
./configure
make
sudo make install
```
