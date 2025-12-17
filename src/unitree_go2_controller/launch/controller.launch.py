#!/usr/bin/env python3
# src/unitree_go2_controller/launch/controller.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package directories
    controller_pkg = get_package_share_directory('unitree_go2_controller')
    
    # Configuration files
    gait_config = PathJoinSubstitution([
        FindPackageShare('unitree_go2_controller'),
        'config',
        'gait.yaml'
    ])
    
    links_config = PathJoinSubstitution([
        FindPackageShare('unitree_go2_controller'),
        'config',
        'links.yaml'
    ])
    
    joints_config = PathJoinSubstitution([
        FindPackageShare('unitree_go2_controller'),
        'config',
        'joints.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50.0',
        description='Controller update rate (Hz)'
    )
    
    # Quadruped controller node
    controller_node = Node(
        package='unitree_go2_controller',
        executable='quadruped_controller_node',
        name='quadruped_controller',
        output='screen',
        parameters=[
            gait_config,
            links_config,
            joints_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'control_rate': LaunchConfiguration('control_rate')
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom/raw')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        control_rate_arg,
        controller_node
    ])
