# Hexapod201 Interface Documentation

This document describes the implementation and usage of the Hexapod201 interface for gait control and body movement.

## Overview

The hexapod interface provides a unified interface for controlling both real and simulated hexapod robots. It supports:

- **Velocity commands**: Integrate velocity commands to generate target poses
- **Direct pose commands**: Move to specific poses directly
- **Gait parameter control**: Configure detailed movement parameters
- **Visualization**: Real-time visualization in RViz
- **Dual mode operation**: Support for both real (PLC) and dummy (simulation) interfaces

## Architecture

### Base Interface (`Hexapod201BaseInterface`)

Abstract base class that defines the common interface for all hexapod controllers:

- **ROS Integration**: Publishers, subscribers, and timers for ROS communication
- **Velocity Integration**: Converts velocity commands to target poses
- **Visualization**: RViz marker visualization of hexapod body
- **State Management**: Tracks current and target poses

### Dummy Interface (`DummyHexapod201Interface`)

Simulation interface for testing and development:

- **Blocking Movement**: Simulates realistic movement with configurable speeds
- **Threaded Execution**: Non-blocking movement execution
- **Interpolation**: Smooth position and orientation interpolation
- **Configurable Parameters**: Adjustable movement and rotation speeds

### Real Interface (`Hexapod201Interface`)

Real robot interface using PLC communication:

- **PLC Connection**: Establishes connection to Beckhoff PLC
- **Parameter Management**: Reads and writes PLC parameters
- **State Monitoring**: Monitors PLC state and feedback
- **Gait Control**: Configures detailed gait parameters

## Installation and Setup

### Prerequisites

- ROS Noetic (or compatible version)
- Python 3
- Required packages:
  - `pyads` (for PLC communication)
  - `numpy`
  - `tf` (for transformations)

### Installation

1. Clone the repository to your ROS workspace
2. Install dependencies:
   ```bash
   pip install pyads numpy
   ```
3. Build the workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Running the Demo

#### Using Launch File (Recommended)

```bash
# Run with dummy interface (simulation)
roslaunch hexapod_controller hexapod_demo.launch use_dummy:=true

# Run with real hexapod (requires PLC connection)
roslaunch hexapod_controller hexapod_demo.launch use_dummy:=false plc_ip:=10.1.180.190.1.1

# Run specific demo type
roslaunch hexapod_controller hexapod_demo.launch demo_type:=velocity
```

#### Using Python Scripts Directly

```bash
# Run hexapod interface only
rosrun hexapod_controller hexapod201_interface.py --dummy

# Run demo with specific type
rosrun hexapod_controller hexapod_gait_demo.py --dummy --demo velocity
```

### Demo Types

1. **Velocity Demo**: Demonstrates velocity command integration

   - Forward/backward movement
   - Turning left/right
   - Sideways movement

2. **Pose Demo**: Demonstrates direct pose commands

   - Move to specific positions
   - Rotate to specific orientations
   - Complex movement sequences

3. **Circle Demo**: Demonstrates continuous movement

   - Circular path following
   - Dynamic orientation control

4. **Gait Parameter Demo**: Demonstrates gait configuration (real hexapod only)

   - Different gait modes
   - Parameter tuning

5. **Interactive Demo**: Interactive command interface
   - User-controlled demos
   - Real-time status monitoring

### Command Line Arguments

#### hexapod201_interface.py

- `--dummy`: Use dummy interface for simulation
- `--plc_ip`: PLC IP address (default: 10.1.180.190.1.1)
- `--node_name`: ROS node name (default: hexapod201_interface)

#### hexapod_gait_demo.py

- `--dummy`: Use dummy interface for simulation
- `--plc_ip`: PLC IP address
- `--demo`: Demo type (velocity, pose, circle, gait, interactive)

## ROS Topics

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/hexapod/pose_cmd` (geometry_msgs/PoseStamped): Direct pose commands

### Published Topics

- `/hexapod/current_pose` (geometry_msgs/PoseStamped): Current hexapod pose
- `/hexapod_visualization` (visualization_msgs/MarkerArray): Visualization markers

## API Reference

### Hexapod201BaseInterface

#### Methods

```python
def move_to_pose(self, target_pose: Pose) -> bool:
    """Move hexapod to target pose - abstract method"""

def setCmd(self, **kwargs) -> bool:
    """Set detailed command parameters - abstract method"""

def stop_movement(self) -> bool:
    """Stop current movement - abstract method"""

def reset_integration(self):
    """Reset velocity integration"""

def get_current_pose(self) -> Pose:
    """Get current pose"""

def get_target_pose(self) -> Pose:
    """Get target pose"""
```

### DummyHexapod201Interface

#### Constructor

```python
def __init__(self, node_name: str = "dummy_hexapod201_interface"):
    """Initialize dummy interface"""
```

#### Configuration

```python
def setCmd(self, **kwargs) -> bool:
    """Set dummy interface parameters

    Args:
        movement_speed: Movement speed in m/s
        rotation_speed: Rotation speed in rad/s
    """
```

### Hexapod201Interface

#### Constructor

```python
def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "10.1.180.190.1.1"):
    """Initialize real hexapod interface"""
```

#### PLC Parameters

```python
def setCmd(self, **kwargs) -> bool:
    """Set detailed movement parameters

    Args:
        TA: Acceleration time (s)
        TM: Movement time (s)
        TD: Deceleration overlap (s)
        TZ: Z advance time (s)
        GaitMode: Gait mode (1=synchronous, 2=tripod, 3=wave)
        GaitDF: Duty factor (0.5-0.75)
        SwapHigh: Swing height (mm)
        ForceMode: Force control mode
    """
```

## Visualization

The interface provides real-time visualization in RViz:

- **Hexapod Body**: Visualized as a cube at the current pose
- **Movement Path**: Shows the trajectory of movement
- **Markers**: Various visualization markers for debugging

### RViz Configuration

The demo includes a pre-configured RViz setup with:

- Grid display
- Marker array visualization
- Pose display
- TF frame visualization

## PLC Communication

### Connection Parameters

- **IP Address**: 10.1.180.190.1.1 (configurable)
- **Port**: 851 (default pyads port)
- **Symbols**: Access to PLC variables for control and feedback

### PLC Variables

- `MAIN.PTCmd.TM`: Time parameters structure
- `MAIN.PTCmd.Gait`: Gait parameters structure
- `MAIN.PTCmd.Pose`: Pose parameters structure
- `MAIN.CtrlCmd`: Control command
- `MAIN.state`: State control
- `MAIN.Q_State`: State feedback
- `MAIN.PTActPos`: Actual position feedback

## Error Handling

The interface includes comprehensive error handling:

- **PLC Connection**: Automatic retry and fallback
- **Movement Execution**: Timeout and error recovery
- **Parameter Validation**: Input validation and bounds checking
- **ROS Communication**: Topic availability checking

## Troubleshooting

### Common Issues

1. **PLC Connection Failed**

   - Check network connectivity
   - Verify PLC IP address
   - Ensure PLC is running and accessible

2. **Movement Not Executing**

   - Check if hexapod is enabled
   - Verify parameter values are within bounds
   - Monitor ROS topics for commands

3. **Visualization Not Working**
   - Ensure RViz is running
   - Check marker topic is being published
   - Verify frame transformations

### Debugging

Enable debug logging:

```bash
export ROS_LOG_LEVEL=DEBUG
```

Monitor topics:

```bash
rostopic echo /hexapod/current_pose
rostopic echo /cmd_vel
```

## Development

### Adding New Features

1. **New Movement Types**: Extend the base interface
2. **Additional Parameters**: Add to setCmd method
3. **Custom Visualizations**: Extend the visualization system

### Testing

- Use dummy interface for development and testing
- Test with real hexapod in controlled environment
- Validate all movement parameters before deployment

## License

This implementation is part of the hexapod controller package and follows the same license terms.

## Contributing

When contributing to this interface:

1. Follow the existing code style
2. Add comprehensive documentation
3. Include unit tests for new features
4. Test with both dummy and real interfaces
5. Update this documentation as needed
