#!/usr/bin/env python3
"""
GPIO Status LED Launch File

Launches the GPIO-based status LED node for R2D2.

This replaces the MCP23017 I2C approach which failed on Jetson AGX Orin
due to 40-pin header I2C pinmux issues.

Usage:
  ros2 launch r2d2_audio gpio_status_led.launch.py
  ros2 launch r2d2_audio gpio_status_led.launch.py simulate:=true

Author: Severin Leuenberger
Date: January 10, 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GPIO status LED node"""
    
    # Declare launch arguments
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable LED control (set to false to disable)'
    )
    
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='Run in simulation mode (no hardware access)'
    )
    
    gesture_flash_duration_arg = DeclareLaunchArgument(
        'gesture_flash_duration_ms',
        default_value='500',
        description='Duration of yellow LED gesture flash in milliseconds'
    )
    
    # GPIO Status LED Node
    gpio_led_node = Node(
            package='r2d2_audio',
            executable='gpio_status_led_node',
            name='gpio_status_led_node',
            output='screen',
            parameters=[{
            'enabled': LaunchConfiguration('enabled'),
            'simulate': LaunchConfiguration('simulate'),
            'gesture_flash_duration_ms': LaunchConfiguration('gesture_flash_duration_ms'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        enabled_arg,
        simulate_arg,
        gesture_flash_duration_arg,
        gpio_led_node,
    ])
