# unitree_go2_msgs

Custom ROS 2 message definitions for the Unitree Go2 robot simulation.

## Overview

This package provides custom message types used by the quadruped controller for body pose control and velocity commands. These messages enable more precise control over the robot's body orientation and movement compared to standard ROS 2 messages.

## Messages

### Pose.msg

6-DOF body pose command for controlling the robot's body position and orientation.

**Fields:**
- `float32 x` - Body X position offset (meters)
- `float32 y` - Body Y position offset (meters)
- `float32 z` - Body height offset from nominal height (meters)
- `float32 roll` - Body roll angle (radians)
- `float32 pitch` - Body pitch angle (radians)
- `float32 yaw` - Body yaw angle (radians)

**Units:**
- Position: meters
- Orientation: radians

**Usage:**
```bash
ros2 topic pub /body_pose unitree_go2_msgs/msg/Pose "{x: 0.0, y: 0.0, z: 0.05, roll: 0.0, pitch: 0.1, yaw: 0.0}"
```

### Velocities.msg

3-DOF velocity command for high-level motion control.

**Fields:**
- `float32 linear_x` - Forward/backward velocity (m/s)
- `float32 linear_y` - Left/right velocity (m/s)
- `float32 angular_z` - Rotational velocity (rad/s)

**Units:**
- Linear velocity: m/s
- Angular velocity: rad/s

**Usage:**
This message type is primarily used internally by the controller. For manual control, use the standard `geometry_msgs/Twist` on the `/cmd_vel` topic instead.

## Integration

These messages are consumed by the `unitree_go2_controller` package:

- **Pose**: Subscribed on `/body_pose` topic for manual body pose adjustments
- **Velocities**: Used internally for gait calculations

## Building

This package is built automatically when you build the entire workspace:

```bash
colcon build --packages-select unitree_go2_msgs
```

After building, source the workspace to use the messages:

```bash
source install/setup.bash
```

## Dependencies

- `std_msgs`
- `geometry_msgs`
- `rosidl_default_generators` (build time)
- `rosidl_default_runtime` (runtime)

## License

BSD-3-Clause - See [LICENSE](../../LICENSE) for details.
