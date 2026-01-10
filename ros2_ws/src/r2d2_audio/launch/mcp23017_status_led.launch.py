#!/usr/bin/env python3
"""
Launch file for MCP23017 Status LED Node

Controls 4 LEDs via MCP23017 I2C GPIO expander:
- Red LED: Person recognized (status="red")
- Blue LED: No person (status="blue")
- Green LED: Unknown person (status="green")
- Yellow LED: Gesture flash (500ms)

Usage:
  # Default settings (I2C bus 1, address 0x20)
  ros2 launch r2d2_audio mcp23017_status_led.launch.py

  # Custom I2C address
  ros2 launch r2d2_audio mcp23017_status_led.launch.py i2c_address:=0x21

  # With reversed wiring (if red wire controls blue LEDs)
  ros2 launch r2d2_audio mcp23017_status_led.launch.py \
    pa0_controls:=blue pa1_controls:=red
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for MCP23017 Status LED node."""
    
    return LaunchDescription([
        # I2C configuration
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number (typically 1 for 40-pin header)'
        ),
        
        DeclareLaunchArgument(
            'i2c_address',
            default_value='32',  # 0x20 in decimal
            description='MCP23017 I2C address (32=0x20, 33=0x21, etc.)'
        ),
        
        # Pin mapping (allows configuration for reversed wiring)
        DeclareLaunchArgument(
            'pa0_controls',
            default_value='red',
            description='Which LED color PA0 controls (red/blue/green/yellow)'
        ),
        
        DeclareLaunchArgument(
            'pa1_controls',
            default_value='blue',
            description='Which LED color PA1 controls (red/blue/green/yellow)'
        ),
        
        DeclareLaunchArgument(
            'pa2_controls',
            default_value='green',
            description='Which LED color PA2 controls (red/blue/green/yellow)'
        ),
        
        DeclareLaunchArgument(
            'pa3_controls',
            default_value='yellow',
            description='Which LED color PA3 controls (red/blue/green/yellow)'
        ),
        
        # Gesture flash duration
        DeclareLaunchArgument(
            'gesture_flash_duration_ms',
            default_value='500',
            description='Yellow LED flash duration in milliseconds'
        ),
        
        # Enable/disable
        DeclareLaunchArgument(
            'enabled',
            default_value='true',
            description='Enable LED controller'
        ),
        
        DeclareLaunchArgument(
            'simulate',
            default_value='false',
            description='Force simulation mode (no hardware required)'
        ),
        
        # MCP23017 Status LED Node
        Node(
            package='r2d2_audio',
            executable='mcp23017_status_led_node',
            name='mcp23017_status_led_controller',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'pa0_controls': LaunchConfiguration('pa0_controls'),
                'pa1_controls': LaunchConfiguration('pa1_controls'),
                'pa2_controls': LaunchConfiguration('pa2_controls'),
                'pa3_controls': LaunchConfiguration('pa3_controls'),
                'gesture_flash_duration_ms': LaunchConfiguration('gesture_flash_duration_ms'),
                'enabled': LaunchConfiguration('enabled'),
                'simulate': LaunchConfiguration('simulate'),
            }]
        ),
    ])

