# unitree_go2_nav2

Complete navigation and simulation launch package for the Unitree Go2 robot with Gazebo Harmonic, SLAM Toolbox, and Nav2.

## Overview

This package provides a complete simulation stack that integrates:
- Gazebo Harmonic physics simulation
- SLAM Toolbox for real-time mapping
- Nav2 for autonomous navigation
- Quadruped locomotion controller
- Pre-configured test environments

The launch file orchestrates a carefully timed startup sequence to ensure all components initialize correctly.

## Quick Start

### Launch Full Simulation

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py
```

This starts:
- Gazebo with Go2 robot (t=0s)
- Robot state publisher (t=3s)
- Quadruped controller (t=5s)
- SLAM Toolbox (t=5s)
- Nav2 stack (t=8s)
- RViz2 visualization (t=8s)

### Set Navigation Goals

1. Wait for the robot to complete startup transition (~10 seconds)
2. In RViz, click "Nav2 Goal" button in toolbar
3. Click and drag on the map to set goal pose
4. Robot will autonomously navigate to the goal

## Launch Arguments

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py [arguments]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo simulation time |
| `use_rviz` | `true` | Launch RViz visualization |
| `use_slam` | `true` | Use SLAM mode (vs localization) |
| `world` | `test_arena` | World file name (without .world) |
| `x_pose` | `0.0` | Initial X position (m) |
| `y_pose` | `0.0` | Initial Y position (m) |
| `z_pose` | `0.39` | Initial Z position (m) |
| `map` | `test_arena.yaml` | Map file (for localization mode) |
| `params_file` | `nav2_sim_params_minimal.yaml` | Nav2 parameters |
| `autostart` | `true` | Auto-start Nav2 lifecycle nodes |

## Usage Examples

### Different Worlds

```bash
# Playground world (open area)
ros2 launch unitree_go2_nav2 simulation.launch.py world:=playground

# Outdoor terrain
ros2 launch unitree_go2_nav2 simulation.launch.py world:=outdoor

# Default simple world
ros2 launch unitree_go2_nav2 simulation.launch.py world:=default
```

### Headless Mode (No RViz)

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py use_rviz:=false
```

Useful for running on remote servers or when you want to connect RViz separately.

### Localization Mode (With Existing Map)

```bash
# First, create a map in SLAM mode, then:
ros2 launch unitree_go2_nav2 simulation.launch.py \
  use_slam:=false \
  map:=/path/to/your_map.yaml
```

### Custom Spawn Position

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py \
  x_pose:=2.0 \
  y_pose:=1.5 \
  z_pose:=0.39
```

## Available Worlds

Located in `worlds/` directory:

| World | Description | Best For |
|-------|-------------|----------|
| `test_arena` | Indoor environment with obstacles | Navigation testing, SLAM |
| `playground` | Open outdoor area | Speed testing, simple navigation |
| `outdoor` | Varied terrain | Robust navigation testing |
| `default` | Simple flat environment | Basic testing, debugging |

## SLAM Workflow

### 1. Build a Map

```bash
# Launch with SLAM enabled (default)
ros2 launch unitree_go2_nav2 simulation.launch.py
```

Drive the robot around using Nav2 goals or manual control to map the environment.

### 2. Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This creates `my_map.yaml` and `my_map.pgm`.

### 3. Use the Map for Localization

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py \
  use_slam:=false \
  map:=$(pwd)/my_map.yaml
```

## Nav2 Configuration

Parameters are in `config/nav2_sim_params_minimal.yaml`.

### Key Settings for Quadrupeds

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      max_vel_x: 0.5          # Maximum forward speed
      max_vel_theta: 0.5      # Maximum rotation speed
      
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.2       # Conservative robot size
      
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.2
```

### Tuning Navigation

**For Faster Navigation:**
- Increase `max_vel_x` and `max_vel_theta`
- Decrease `controller_frequency` if CPU limited

**For Better Obstacle Avoidance:**
- Increase `robot_radius` in costmaps
- Adjust `inflation_radius` in costmap parameters

**For Slower, More Stable Navigation:**
- Decrease velocity limits
- Increase `transform_tolerance` if TF errors occur

## Startup Sequence

The launch file uses a carefully timed sequence:

1. **t=0s**: Gazebo starts, ROS-Gazebo bridge initialized
2. **t=3s**: Robot spawned in simulation
3. **t=5s**: Quadruped controller starts, SLAM begins
4. **t=8-9s**: Joint controllers spawn
5. **t=8s**: Nav2 stack and RViz launch

This timing ensures:
- Gazebo is ready before spawning robot
- Robot description published before controller needs it
- TF tree stable before SLAM starts
- Controller running before Nav2 needs odometry

## Configuration Files

### Gazebo Bridge (`config/gz_bridge.yaml`)

Bridges Gazebo topics to ROS 2:
- `/clock`: Simulation time
- `/scan`: LiDAR data
- `/imu`: IMU data
- Joint states and commands

### Kinematics Config (`config/kinematics_config.yaml`)

Quadruped-specific parameters:
- Leg joint mappings
- Gait configuration
- Velocity limits
- Body dimensions

### Nav2 Parameters (`config/nav2_sim_params_minimal.yaml`)

Complete Nav2 configuration:
- DWB local planner settings
- Costmap parameters
- SLAM Toolbox configuration
- Behavior tree settings

## Troubleshooting

### Simulation Won't Start

1. Check Gazebo Harmonic is installed:
   ```bash
   gz sim --version
   ```

2. Verify all dependencies are installed (see main README)

### Robot Doesn't Move

1. Check controller is running:
   ```bash
   ros2 node list | grep quadruped_controller
   ```

2. Verify Nav2 is publishing commands:
   ```bash
   ros2 topic echo /cmd_vel
   ```

3. Check for errors in controller logs:
   ```bash
   ros2 node info /quadruped_controller
   ```

### SLAM Issues

**Poor Map Quality:**
- Drive slower to allow SLAM to process scans
- Ensure good LiDAR coverage of environment
- Check `/scan` topic is publishing: `ros2 topic hz /scan`

**Transform Errors:**
Increase `transform_tolerance` in Nav2 config:
```yaml
bt_navigator:
  ros__parameters:
    transform_tolerance: 3.0  # Increase from 2.0
```

### Navigation Failures

**Robot Gets Stuck:**
- Increase `robot_radius` in costmap config
- Adjust recovery behavior timeouts
- Check costmap inflation settings

**Jerky Motion:**
- Reduce controller frequency
- Increase velocity smoothing
- Check control loop rate is stable

### Performance Issues

**Low Frame Rate:**
- Launch without RViz: `use_rviz:=false`
- Reduce Nav2 update frequencies
- Use simpler world file

**High CPU Usage:**
- Reduce SLAM resolution
- Lower controller frequency
- Disable unnecessary sensors

## Creating Custom Worlds

1. Create new `.world` file in `worlds/` directory
2. Follow Gazebo Harmonic SDF format
3. Include ground plane and lighting
4. Add models and obstacles

Example:
```bash
ros2 launch unitree_go2_nav2 simulation.launch.py world:=my_custom_world
```

## Development

### Building

```bash
colcon build --packages-select unitree_go2_nav2
```

### Modifying Launch Sequence

Edit `launch/simulation.launch.py`:
- Adjust `TimerAction` periods to change startup timing
- Add/remove components as needed
- Modify launch arguments

### Testing Navigation Parameters

1. Modify `config/nav2_sim_params_minimal.yaml`
2. Rebuild: `colcon build --packages-select unitree_go2_nav2`
3. Relaunch simulation
4. Monitor performance and adjust

## ROS Topics

Key topics during simulation:

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands to robot |
| `/odom` | Odometry | Robot odometry |
| `/scan` | LaserScan | LiDAR data |
| `/map` | OccupancyGrid | SLAM-generated map |
| `/tf`, `/tf_static` | TFMessage | Transform tree |
| `/joint_states` | JointState | Joint positions |

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2 stack
- SLAM Toolbox
- ros_gz packages
- unitree_go2_controller
- unitree_go2_description

## License

BSD-3-Clause - See [LICENSE](../../LICENSE) for details.
