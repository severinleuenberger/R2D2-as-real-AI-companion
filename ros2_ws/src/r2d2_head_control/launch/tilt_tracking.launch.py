#!/usr/bin/env python3
"""
Launch file for tilt tracking node.

Launches the face-tracking tilt servo controller with parameters from YAML config.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('r2d2_head_control')
    
    # Path to default parameters file
    default_params_file = os.path.join(pkg_share, 'config', 'tilt_tracking_params.yaml')
    
    # Declare launch arguments
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable/disable tilt tracking'
    )
    
    simulate_arg = DeclareLaunchArgument(
        'simulate_hardware',
        default_value='false',
        description='Run in simulation mode without hardware'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameters YAML file'
    )
    
    # Create tilt tracking node
    tilt_tracking_node = Node(
        package='r2d2_head_control',
        executable='tilt_tracking_node',
        name='tilt_tracking_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enabled': LaunchConfiguration('enabled'),
                'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        enabled_arg,
        simulate_arg,
        params_file_arg,
        tilt_tracking_node,
    ])

