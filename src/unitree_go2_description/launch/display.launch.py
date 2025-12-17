#!/usr/bin/env python3
# src/unitree_go2_description/launch/display.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('unitree_go2_description')
    
    # Path to main xacro file (this will be processed in real-time)
    xacro_file = os.path.join(pkg_share, 'xacro', 'unitree_go2.xacro')
    
    # Default RViz config
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'unitree_go2.rviz')
    
    # If the config doesn't exist, don't specify one
    rviz_config_arg = default_rviz_config if os.path.exists(default_rviz_config) else ''
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default=rviz_config_arg)
    
    # Process the xacro file to generate robot_description
    # This ensures we're always using the latest xacro definitions
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0
        }]
    )
    
    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz Node
    rviz_args = ['-d', rviz_config] if rviz_config_arg else []
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_arg,
            description='Path to RViz config file'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
