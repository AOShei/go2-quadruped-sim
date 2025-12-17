# unitree_go2_description

URDF/xacro robot description package for the Unitree Go2 quadruped robot, compatible with Gazebo Harmonic and ROS 2 Jazzy.

## Overview

This package contains the complete kinematic and dynamic model of the Unitree Go2 robot, including:
- Modular xacro-based URDF structure
- Visual and collision geometry (DAE meshes)
- Gazebo Harmonic physics plugins
- Sensor definitions (IMU, 2D LiDAR)
- ros2_control integration

## Features

- **12 Degrees of Freedom**: 4 legs × 3 joints (hip, thigh, calf)
- **Realistic Physics**: Mass ~25 kg with accurate inertia tensors
- **Sensors**:
  - IMU (acceleration, angular velocity, orientation)
  - 2D LiDAR (270° field of view, 10m range)
  - Joint state sensing
- **Gazebo Integration**: Full gz_ros2_control support
- **RViz Visualization**: Pre-configured RViz settings

## Quick Start

### Visualize in RViz

Launch the robot model with joint state publisher GUI:

```bash
ros2 launch unitree_go2_description display.launch.py
```

This opens RViz with the robot model and a GUI to control joint angles.

### Generate URDF from Xacro

```bash
ros2 run xacro xacro src/unitree_go2_description/xacro/unitree_go2.xacro > go2.urdf
```

### Validate URDF

```bash
check_urdf go2.urdf
```

## Robot Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Total Mass | ~25 kg |
| Body Length | 0.645 m |
| Body Width | 0.28 m |
| Default Height | 0.30 m |
| Leg Reach | ~0.426 m |

### Joint Limits

| Joint Type | Position Limit | Velocity Limit | Torque Limit |
|------------|----------------|----------------|--------------|
| Hip (abduction) | ±60° | 30.1 rad/s | 23.7 Nm |
| Thigh | -48° to 206° | 30.1 rad/s | 23.7 Nm |
| Calf (knee) | -154° to -52° | 20.0 rad/s | 35.55 Nm |

### Sensors

**IMU** (`imu_link`):
- Position: [-0.02557, 0, 0.04232] relative to trunk
- Publishes: `/imu` topic
- Data: Linear acceleration, angular velocity, orientation

**LiDAR** (`lidar_link`):
- Type: 2D scanning laser
- FOV: 270° (-135° to +135°)
- Range: 0.1m to 10.0m
- Resolution: 720 samples
- Publishes: `/scan` topic

## File Structure

```
unitree_go2_description/
├── config/
│   └── ros_control.yaml          # Controller configuration
├── launch/
│   └── display.launch.py         # RViz visualization launch
├── meshes/
│   ├── trunk.dae                 # Body mesh
│   ├── hip.dae                   # Hip joint mesh
│   ├── thigh.dae / thigh_mirror.dae
│   ├── calf.dae / calf_mirror.dae
│   └── foot.dae                  # Foot mesh
├── rviz/
│   └── unitree_go2.rviz          # RViz configuration
└── xacro/
    ├── unitree_go2.xacro         # Main robot definition
    ├── constants.xacro           # Physical constants
    ├── leg.xacro                 # Leg macro definition
    ├── gazebo.xacro              # Gazebo plugins
    ├── lidar.xacro               # LiDAR sensor
    └── materials.xacro           # Visual materials
```

## Modifying the Robot

### Adjusting Physical Parameters

Edit `xacro/constants.xacro` to modify:
- Link lengths and offsets
- Joint limits
- Mass and inertia properties
- Damping and friction

Example:
```xml
<xacro:property name="thigh_length" value="0.213"/>
<xacro:property name="calf_length" value="0.213"/>
<xacro:property name="nominal_height" value="0.30"/>
```

### Adding New Sensors

1. Create a new xacro file in `xacro/` directory
2. Define the sensor link and joint
3. Add Gazebo plugin configuration
4. Include the file in `unitree_go2.xacro`

### Debug Mode

The URDF supports a DEBUG mode that fixes the robot to the world frame for kinematic testing:

```bash
ros2 run xacro xacro src/unitree_go2_description/xacro/unitree_go2.xacro DEBUG:=true
```

This is useful for testing joint movements without physics simulation.

## TF Frame Tree

```
base_link (root)
└─ trunk
   ├─ imu_link
   ├─ lidar_link
   ├─ FL_hip → FL_thigh → FL_calf → FL_foot
   ├─ FR_hip → FR_thigh → FR_calf → FR_foot
   ├─ RL_hip → RL_thigh → RL_calf → RL_foot
   └─ RR_hip → RR_thigh → RR_calf → RR_foot
```

**Frame Convention:**
- FL = Front Left
- FR = Front Right
- RL = Rear Left
- RR = Rear Right

## Gazebo Integration

The robot uses the following Gazebo plugins:

- **gz_ros2_control**: Joint state and command interface
- **IMU Sensor**: Publishes IMU data
- **LiDAR Sensor**: Publishes laser scan data
- **Joint State Publisher**: Publishes joint states for visualization

## Coordinate System

- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up

All measurements follow REP-103 (Standard Units of Measure and Coordinate Conventions).

## Dependencies

- `ament_cmake`
- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `gz_ros2_control`
- `controller_manager`

## Validation

The package includes a validation test:

```bash
colcon build --packages-select unitree_go2_description --cmake-target validate_xacro
```

This ensures the xacro files generate valid URDF.

## License

BSD-3-Clause - See [LICENSE](../../LICENSE) for details.

Robot meshes are from Unitree Robotics, licensed under BSD-3-Clause.
