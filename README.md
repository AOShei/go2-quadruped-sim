# Unitree Go2 ROS 2 Jazzy Simulation

A complete ROS 2 simulation package for the Unitree Go2 quadruped robot, featuring autonomous navigation, SLAM mapping, and a CHAMP-based locomotion controller. Designed for educational use and as a foundation for robotics research and development.

## Features

- **Full Gazebo Harmonic Simulation**: Physics-accurate simulation with Gazebo Harmonic (gz-sim)
- **CHAMP Locomotion Controller**: Quadruped gait generation with inverse kinematics
- **Nav2 Integration**: Autonomous navigation with dynamic path planning
- **SLAM Mapping**: Real-time map building with SLAM Toolbox
- **Multiple Test Environments**: Pre-configured worlds for different scenarios
- **ROS 2 Jazzy**: Built for the latest ROS 2 LTS release

## System Requirements

- **Operating System**: Ubuntu 24.04 (Noble Numbat)
- **ROS Distribution**: ROS 2 Jazzy Jalisco
- **Simulator**: Gazebo Harmonic
- **Hardware**: Recommended 8GB RAM, 4+ CPU cores

## Installation

### 1. Install ROS 2 Jazzy

If you haven't installed ROS 2 Jazzy yet, follow the [official installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
```

### 3. Install Dependencies

```bash
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-robot-localization \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  python3-colcon-common-extensions
```

### 4. Build the Workspace

```bash
# Clone the repository
cd ~
git clone <repository-url> unitree_go2_ros2_jazzy
cd unitree_go2_ros2_jazzy

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Quick Start

### Launch Full Simulation

```bash
source install/setup.bash
ros2 launch unitree_go2_nav2 simulation.launch.py
```

This will start:
- Gazebo Harmonic with the Go2 robot
- SLAM Toolbox for mapping
- Nav2 for autonomous navigation
- RViz2 for visualization

### Send Navigation Goals

In RViz2, use the "Nav2 Goal" button to set navigation waypoints. The robot will autonomously plan and execute paths to reach the goal.

### Control the Robot Manually

```bash
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

## Package Overview

This workspace contains four ROS 2 packages:

### [unitree_go2_description](src/unitree_go2_description/)
Robot URDF/xacro model with Gazebo Harmonic integration.
- Complete kinematic model (12 DOF)
- Sensors: IMU, 2D LiDAR (270°)
- Realistic mass and inertia properties

### [unitree_go2_controller](src/unitree_go2_controller/)
CHAMP-based quadruped locomotion controller.
- Inverse kinematics solver
- Gait generation (trot, walk, etc.)
- Body pose stabilization
- Odometry estimation

### [unitree_go2_nav2](src/unitree_go2_nav2/)
Navigation and simulation launch files.
- Full simulation stack integration
- SLAM and localization modes
- Multiple pre-configured worlds
- Nav2 parameter tuning for quadrupeds

### [unitree_go2_msgs](src/unitree_go2_msgs/)
Custom message definitions.
- `Pose`: 6-DOF body pose commands
- `Velocities`: 3-DOF velocity commands

## Usage Examples

### Launch with Different Worlds

```bash
# Playground environment
ros2 launch unitree_go2_nav2 simulation.launch.py world:=playground

# Outdoor terrain
ros2 launch unitree_go2_nav2 simulation.launch.py world:=outdoor

# Default simple world
ros2 launch unitree_go2_nav2 simulation.launch.py world:=default
```

### Launch Without RViz

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py use_rviz:=false
```

### Use Localization Mode (Requires Existing Map)

```bash
ros2 launch unitree_go2_nav2 simulation.launch.py use_slam:=false map:=/path/to/map.yaml
```

### Visualize Robot Model Only

```bash
ros2 launch unitree_go2_description display.launch.py
```

### Save a Map

After running SLAM and mapping an environment:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Development

### Build Individual Packages

```bash
colcon build --packages-select unitree_go2_description
colcon build --packages-select unitree_go2_controller
colcon build --packages-select unitree_go2_msgs
colcon build --packages-select unitree_go2_nav2
```

### View Topics

```bash
# List all active topics
ros2 topic list

# Monitor odometry
ros2 topic echo /odom

# Monitor laser scans
ros2 topic echo /scan
```

### Adjust Gait Parameters

Edit `src/unitree_go2_controller/config/kinematics_config.yaml`:

```yaml
gait.swing_height: 0.04      # Foot lift height (meters)
gait.stance_duration: 0.25   # Stance phase duration (seconds)
gait.nominal_height: 0.225   # Default body height (meters)
```

## Documentation

- **[CLAUDE.md](CLAUDE.md)**: Technical documentation for Claude Code AI assistant
- **[CREDITS.md](CREDITS.md)**: Attribution and licensing information
- **Package READMEs**: See individual package directories for detailed documentation

## Troubleshooting

### Robot Doesn't Move

1. Check if the controller is running:
   ```bash
   ros2 node list | grep quadruped_controller
   ```

2. Verify velocity commands are being received:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### Poor SLAM Performance

Increase the transform tolerance in `src/unitree_go2_nav2/config/nav2_sim_params_minimal.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    transform_tolerance: 3.0  # Increase if needed
```

### Gazebo Crashes or Won't Start

Ensure Gazebo Harmonic is properly installed:

```bash
gz sim --version  # Should show Gazebo Harmonic
```

### Build Errors

Clean and rebuild:

```bash
rm -rf build/ install/ log/
colcon build
```

## Contributing

Contributions are welcome! This project is designed for educational use and as a jumping-off point for further development.

### Potential Enhancements

- Additional gait patterns
- Terrain adaptation algorithms
- Advanced sensor integration
- Real robot hardware interface
- Multi-robot simulation

## License

This project is licensed under the **BSD-3-Clause License**. See [LICENSE](LICENSE) for details.

### Third-Party Acknowledgments

This project incorporates code from:
- **CHAMP** by Juan Miguel Jimeno (BSD-3-Clause)
- **Unitree Go2** robot assets from Unitree Robotics (BSD-3-Clause)

See [CREDITS.md](CREDITS.md) for complete attribution information.

## Citation

If you use this simulation in your research, please cite:

```bibtex
@software{unitree_go2_ros2,
  author = {O'Shei, Andrew},
  title = {Unitree Go2 ROS 2 Jazzy Simulation},
  year = {2024-2025},
  url = {<repository-url>}
}
```

## Contact

**Maintainer**: Andrew O'Shei  
**Email**: andrewoshei@gmail.com

## Acknowledgments

Special thanks to:
- **Juan Miguel Jimeno** for the CHAMP quadruped controller framework
- **Unitree Robotics** for the Go2 robot and specifications
- The **ROS 2** and **Gazebo** communities

---

**Educational Use**: This simulation is specifically designed for learning robotics, testing algorithms, and developing new features for quadruped robots.
