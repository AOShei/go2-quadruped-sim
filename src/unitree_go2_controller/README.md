# unitree_go2_controller

CHAMP-based quadruped locomotion controller for the Unitree Go2 robot, adapted for ROS 2 Jazzy.

## Overview

This package provides a complete locomotion controller for quadruped robots, featuring:
- Inverse kinematics solver for 3-joint legs
- Gait generation and trajectory planning
- Body pose stabilization
- Odometry estimation from joint states

The controller is based on the [CHAMP](https://github.com/chvmp/champ) (CHAMPionship quadruped controller) framework, heavily modified for ROS 2 Jazzy and optimized for the Unitree Go2's kinematic structure.

## Features

- **50 Hz Control Loop**: Real-time joint command generation
- **Configurable Gaits**: Trot, walk, and custom gait patterns
- **Body Height Control**: Dynamic body height adjustment (0.15m - 0.35m)
- **Omnidirectional Motion**: Forward, lateral, and rotational movement
- **Smooth Startup**: Gradual transition from spawn pose to standing stance
- **IK-based Control**: Foot position commands converted to joint angles

## Quick Start

### Launch Controller (Standalone)

```bash
ros2 launch unitree_go2_controller controller.launch.py
```

**Note**: This requires `robot_description` and `joint_states` to be published (normally done by the simulation).

### Send Velocity Commands

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Adjust Body Pose

```bash
# Raise body height by 5cm
ros2 topic pub /body_pose unitree_go2_msgs/msg/Pose "{z: 0.05}"

# Tilt body forward
ros2 topic pub /body_pose unitree_go2_msgs/msg/Pose "{pitch: 0.1}"
```

## ROS Interface

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (m/s, rad/s) |
| `/body_pose` | `unitree_go2_msgs/Pose` | Body pose adjustments (m, rad) |
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions |

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/joint_group_effort_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 50 Hz | Joint position commands |
| `/odom/raw` | `nav_msgs/Odometry` | 50 Hz | Odometry estimates |

### Parameters

Configure in `config/kinematics_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gait.nominal_height` | 0.225 m | Default body height above ground |
| `gait.swing_height` | 0.04 m | Foot lift height during swing phase |
| `gait.stance_duration` | 0.25 s | Duration of stance phase |
| `gait.max_linear_velocity_x` | 0.3 m/s | Maximum forward speed |
| `gait.max_linear_velocity_y` | 0.25 m/s | Maximum lateral speed |
| `gait.max_angular_velocity_z` | 0.5 rad/s | Maximum rotation speed |
| `control_rate` | 50.0 Hz | Controller update frequency |

## Configuration

### Tuning Gaits

Edit `config/kinematics_config.yaml`:

```yaml
quadruped_controller:
  ros__parameters:
    # Gait parameters
    gait.swing_height: 0.04          # Increase for rough terrain
    gait.stance_duration: 0.25       # Decrease for faster gaits
    gait.nominal_height: 0.225       # Adjust body height
```

**Gait Tuning Tips:**
- **Rough Terrain**: Increase `swing_height` to 0.05-0.06m
- **Faster Speed**: Decrease `stance_duration` to 0.20s
- **Stability**: Increase `stance_duration` to 0.30s, lower body height
- **Climbing**: Increase `swing_height`, slow velocity limits

### Joint Mapping

The controller expects joints in this order (matching URDF):

```
FL_hip_joint, FL_thigh_joint, FL_calf_joint,  # Front Left
FR_hip_joint, FR_thigh_joint, FR_calf_joint,  # Front Right
RL_hip_joint, RL_thigh_joint, RL_calf_joint,  # Rear Left
RR_hip_joint, RR_thigh_joint, RR_calf_joint   # Rear Right
```

## Architecture

### Control Flow

1. **Input**: Receive velocity commands (`/cmd_vel`) and body pose (`/body_pose`)
2. **Gait Generation**: Leg controller generates foot trajectories based on gait pattern
3. **Body Controller**: Applies body pose offsets to foot positions
4. **Inverse Kinematics**: Convert foot positions to joint angles
5. **Output**: Publish joint commands to `joint_group_effort_controller`

### CHAMP Components

The controller uses these CHAMP modules (in `include/champ/`):

- **QuadrupedBase**: Robot model with leg and joint definitions
- **LegController**: Gait generation and phase management
- **BodyController**: Body pose control and stabilization
- **Kinematics**: Forward and inverse kinematics solver
- **Odometry**: Odometry estimation from joint states

## Coordinate System

**Body Frame:**
- X: Forward
- Y: Left
- Z: Up

**Gait Phases:**
- Stance: Foot on ground, propelling robot
- Swing: Foot in air, repositioning for next stance

## Troubleshooting

### Robot Falls or Doesn't Stand

1. Check nominal height setting:
   ```bash
   ros2 param get /quadruped_controller gait.nominal_height
   ```
   Should be ~0.225m for Go2.

2. Verify joint states are being received:
   ```bash
   ros2 topic hz /joint_states
   ```
   Should be >10 Hz.

### Robot Doesn't Respond to Commands

1. Check if controller node is running:
   ```bash
   ros2 node list | grep quadruped_controller
   ```

2. Verify cmd_vel is being received:
   ```bash
   ros2 topic echo /cmd_vel
   ```

3. Check for IK failures in logs:
   ```bash
   ros2 node info /quadruped_controller
   ```

### Unstable Walking

1. Reduce maximum velocities in config file
2. Increase stance duration for more stable gait
3. Lower body height for better stability
4. Check ground contact in Gazebo

### Joint Position Jumps

This usually indicates IK solver issues. Check:
- Foot positions are within reachable workspace
- Joint limits are properly configured
- Robot URDF matches kinematics configuration

## Development

### Building

```bash
colcon build --packages-select unitree_go2_controller
```

### Running with Debug Logging

```bash
ros2 run unitree_go2_controller quadruped_controller_node --ros-args --log-level DEBUG
```

### Modifying Kinematics

1. Edit `include/go2_quadruped_description.h` for leg geometry
2. Update `include/champ/kinematics/kinematics.h` for IK solver
3. Rebuild and test in simulation

## Dependencies

- `rclcpp`
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `trajectory_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `unitree_go2_msgs`
- `controller_interface`, `hardware_interface`
- `urdf`

## Attribution

This controller is based on the [CHAMP](https://github.com/chvmp/champ) quadruped controller framework by Juan Miguel Jimeno (BSD-3-Clause license).

**Modifications for Go2:**
- Updated for ROS 2 Jazzy API
- Adapted kinematics for 3-joint leg configuration
- Modified coordinate transformations for Go2 joint conventions
- Integrated with ros2_control
- Tuned gait parameters for Go2 mass and dimensions

See [CREDITS.md](../../CREDITS.md) for full attribution.

## License

BSD-3-Clause - See [LICENSE](../../LICENSE) for details.
