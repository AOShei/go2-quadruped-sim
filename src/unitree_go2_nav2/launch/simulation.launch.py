#!/usr/bin/env python3
# src/unitree_go2_nav2/launch/simulation/go2_full_sim_launch.py
# Complete Unitree Go2 Simulation with Gazebo Harmonic, SLAM Toolbox, and Nav2

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # ===========================================
    # 1. SETUP AND ARGUMENTS
    # ===========================================
    go2_nav2_dir = get_package_share_directory('unitree_go2_nav2')
    go2_description_dir = get_package_share_directory('unitree_go2_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Environment Variable Setup for Gazebo resources
    gz_model_path = os.path.dirname(go2_description_dir)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_model_path
    )

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    world_name = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Define Arguments
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'use_rviz', 
            default_value='true', 
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'use_slam', 
            default_value='true', 
            description='Use SLAM Toolbox instead of localization'
        ),
        DeclareLaunchArgument(
            'autostart', 
            default_value='true', 
            description='Automatically startup lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'world', 
            default_value='test_arena', 
            description='World file name (without extension)'
        ),
        DeclareLaunchArgument(
            'x_pose', 
            default_value='0.0', 
            description='Initial x position'
        ),
        DeclareLaunchArgument(
            'y_pose', 
            default_value='0.0', 
            description='Initial y position'
        ),
        DeclareLaunchArgument(
            'z_pose', 
            default_value='0.39', 
            description='Initial z position (height above ground)'
        ),
        DeclareLaunchArgument(
            'map', 
            default_value=os.path.join(go2_nav2_dir, 'maps', 'test_arena.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(go2_nav2_dir, 'config', 'nav2_sim_params_minimal.yaml'),
            description='Full path to Nav2 parameters file'
        ),
    ]

    # ===========================================
    # 2. ROBOT DESCRIPTION (URDF)
    # ===========================================
    
    # Use our new Gazebo Harmonic compatible URDF
    urdf_file = PathJoinSubstitution([
        FindPackageShare('unitree_go2_description'), 
        'xacro', 
        'unitree_go2.xacro'
    ])
    
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    # Controller spawner nodes
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # This is the important controller that receives commands from your custom node
    spawn_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    kinematics_params_file = os.path.join(go2_nav2_dir, 'config', 'kinematics_config.yaml')

    go2_controller_node = Node(
        package='unitree_go2_controller',
        executable='quadruped_controller_node',
        name='quadruped_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            kinematics_params_file
        ],
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=8.0, # Increased from 6.0
        actions=[spawn_joint_state_broadcaster]
    )

    delayed_joint_controller = TimerAction(
        period=9.0, # Increased from 7.0
        actions=[spawn_joint_controller]
    )

    delayed_go2_controller = TimerAction(
        period=5.0, # Adjusted from 4.0
        actions=[go2_controller_node]
    )

    # ===========================================
    # 3. GAZEBO HARMONIC
    # ===========================================
    
    # Start Gazebo Harmonic (gz sim)
    world_file = PathJoinSubstitution([
        FindPackageShare('unitree_go2_nav2'), 
        'worlds', 
        [world_name, '.world']  # Note: Gazebo Harmonic uses .world files
    ])
    
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'}
    )

    # ===========================================
    # 4. ROS-GAZEBO BRIDGE
    # ===========================================
    
    # Bridge configuration file
    bridge_config = os.path.join(go2_nav2_dir, 'config', 'gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ===========================================
    # 5. SPAWN ROBOT
    # ===========================================
    
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'go2',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )
    
    # Group robot spawning with delay
    spawn_group = GroupAction(
        actions=[
            robot_state_publisher_node,
            spawn_node,
        ]
    )
    
    # Delay spawn to let Gazebo initialize
    spawn_delay = TimerAction(
        period=3.0,
        actions=[spawn_group]
    )

    # ===========================================
    # 6. SLAM TOOLBOX
    # ===========================================

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_slam)
    )
    
    # SLAM Lifecycle Manager
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['slam_toolbox']}
        ],
        condition=IfCondition(use_slam)
    )
    
    # Group SLAM components
    slam_group = GroupAction(
        actions=[
            slam_toolbox_node,
            lifecycle_manager_slam,
        ]
    )
    
    # Delay SLAM to let robot spawn and TF stabilize
    delayed_slam = TimerAction(
        period=5.0,  # Start 2 seconds after robot spawn
        actions=[slam_group]
    )
    
    # ===========================================
    # 7. NAV2 STACK
    # ===========================================
    
    # Nav2 for SLAM mode (no AMCL)
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items(),
        condition=IfCondition(use_slam)
    )

    # Nav2 for localization mode (with AMCL)
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml,
            'autostart': autostart
        }.items(),
        condition=UnlessCondition(use_slam)
    )
    
    # Group Nav2 components
    nav2_group = GroupAction(
        actions=[
            nav2_slam_launch,
            nav2_localization_launch,
        ]
    )

    # Delay Nav2 to let SLAM initialize
    delayed_nav2 = TimerAction(
        period=8.0,  # Start 3 seconds after SLAM (total 11s from start)
        actions=[nav2_group]
    )
    
    # ===========================================
    # 9. RVIZ
    # ===========================================
    
    rviz_config = os.path.join(go2_nav2_dir, 'config', 'go2_nav2_sim.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Delay RViz
    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz_node]
    )

    # ===========================================
    # 10. LAUNCH DESCRIPTION
    # ===========================================
    
    return LaunchDescription(
        declare_args + [
            # Environment
            gz_resource_path,
            
            # Start Gazebo and bridge (t=0s)
            start_gazebo_cmd,
            ros_gz_bridge,
            
            # Spawn robot (t=3s)
            spawn_delay,
            
            # Start custom kinematics controller (t=4s)
            delayed_go2_controller,
            
            # Start SLAM (t=5s)
            delayed_slam,
            
            # Start joint state controllers (t=[6s,7s])
            delayed_joint_state_broadcaster,
            delayed_joint_controller,
            
            # Start Nav2 (t=8s)
            delayed_nav2,
            
            # Start RViz (t=8s)
            delayed_rviz,
        ]
    )
