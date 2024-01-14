# fast_legged_planner

Test repo for Fast-Legged-Planner

## Setup

### Robotic Course Setup

Create and activate conda environment:

```bash
conda env create -f robotic_course_env.yml
```

### Deps(TODO)

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
