#!/usr/bin/env python3
"""
Launch file for gesture intent node.

Launches the gesture intent node that translates gesture events
into conversation start/stop actions with strict gating and
automatic speech service shutdown watchdog.

Usage:
    ros2 launch r2d2_gesture gesture_intent.launch.py

Parameters:
    cooldown_start_seconds: float (default: 2.0)
    cooldown_stop_seconds: float (default: 1.0)
    enabled: bool (default: true)
    auto_shutdown_enabled: bool (default: true)
    auto_shutdown_timeout_seconds: float (default: 300.0 = 5 minutes)
    auto_restart_on_return: bool (default: false)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for gesture intent node."""
    
    # Declare launch arguments
    cooldown_start_arg = DeclareLaunchArgument(
        'cooldown_start_seconds',
        default_value='2.0',
        description='Cooldown period after start gesture (seconds)'
    )
    
    cooldown_stop_arg = DeclareLaunchArgument(
        'cooldown_stop_seconds',
        default_value='1.0',
        description='Cooldown period after stop gesture (seconds)'
    )
    
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable/disable gesture intent node'
    )
    
    auto_shutdown_enabled_arg = DeclareLaunchArgument(
        'auto_shutdown_enabled',
        default_value='true',
        description='Enable automatic speech shutdown when person absent'
    )
    
    auto_shutdown_timeout_arg = DeclareLaunchArgument(
        'auto_shutdown_timeout_seconds',
        default_value='300.0',
        description='Seconds before auto-shutdown (default: 5 minutes)'
    )
    
    auto_restart_arg = DeclareLaunchArgument(
        'auto_restart_on_return',
        default_value='false',
        description='Auto-restart speech when person returns'
    )
    
    # Create gesture intent node
    gesture_intent_node = Node(
        package='r2d2_gesture',
        executable='gesture_intent_node',
        name='gesture_intent_node',
        output='screen',
        parameters=[{
            'cooldown_start_seconds': LaunchConfiguration('cooldown_start_seconds'),
            'cooldown_stop_seconds': LaunchConfiguration('cooldown_stop_seconds'),
            'enabled': LaunchConfiguration('enabled'),
            'auto_shutdown_enabled': LaunchConfiguration('auto_shutdown_enabled'),
            'auto_shutdown_timeout_seconds': LaunchConfiguration('auto_shutdown_timeout_seconds'),
            'auto_restart_on_return': LaunchConfiguration('auto_restart_on_return'),
        }]
    )
    
    return LaunchDescription([
        cooldown_start_arg,
        cooldown_stop_arg,
        enabled_arg,
        auto_shutdown_enabled_arg,
        auto_shutdown_timeout_arg,
        auto_restart_arg,
        gesture_intent_node,
    ])

