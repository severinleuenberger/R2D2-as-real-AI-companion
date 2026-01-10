#!/usr/bin/env python3
"""
Launch file for all R2D2 audio services.

Launches:
  1. audio_notification_node - Core state machine and audio alerts
  2. status_led_node - Visual feedback via LED
  3. database_logger_node - Event logging (structure ready for DB)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for all audio services."""
    
    return LaunchDescription([
        # Arguments (can be overridden from command line)
        DeclareLaunchArgument(
            'target_person',
            default_value='target_person',
            description='Name of person to recognize (auto-resolved from PersonRegistry)'
        ),
        DeclareLaunchArgument(
            'audio_volume',
            default_value='0.05',
            description='Audio alert volume (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'jitter_tolerance_seconds',
            default_value='5.0',
            description='Brief gap tolerance (seconds)'
        ),
        DeclareLaunchArgument(
            'recognition_window_seconds',
            default_value='15.0',
            description='Minimum recognition state duration (seconds)'
        ),
        DeclareLaunchArgument(
            'enable_led',
            default_value='true',
            description='Enable status LED controller'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Enable database logger'
        ),
        DeclareLaunchArgument(
            'gesture_flash_duration_ms',
            default_value='500',
            description='Yellow LED flash duration for gestures (milliseconds)'
        ),
        DeclareLaunchArgument(
            'simulate_led',
            default_value='false',
            description='Simulate GPIO LEDs if no hardware available'
        ),
        
        # Core Audio Notification Node
        Node(
            package='r2d2_audio',
            executable='audio_notification_node',
            name='audio_notification_node',
            output='screen',
            parameters=[
                {
                    'target_person': LaunchConfiguration('target_person'),
                    'audio_volume': LaunchConfiguration('audio_volume'),
                    'jitter_tolerance_seconds': LaunchConfiguration('jitter_tolerance_seconds'),
                    'loss_confirmation_seconds': LaunchConfiguration('recognition_window_seconds'),
                    'cooldown_seconds': 2.0,
                    'recognition_audio_file': 'Voicy_R2-D2 - 2.mp3',
                    'loss_audio_file': 'Voicy_R2-D2 - 5.mp3',
                    'enabled': True,
                }
            ],
        ),
        
        # GPIO Status LED Controller (using transistors on Pins 7, 11, 13)
        Node(
            package='r2d2_audio',
            executable='gpio_status_led_node',
            name='gpio_status_led_node',
            output='screen',
            parameters=[
                {
                    'gesture_flash_duration_ms': LaunchConfiguration('gesture_flash_duration_ms'),
                    'enabled': LaunchConfiguration('enable_led'),
                    'simulate': LaunchConfiguration('simulate_led'),
                }
            ],
        ),
        
        # Database Logger Node
        Node(
            package='r2d2_audio',
            executable='database_logger_node',
            name='database_logger',
            output='screen',
            parameters=[
                {
                    'db_path': os.path.expanduser('~/dev/r2d2/r2d2_conversations.db'),
                    'enabled': LaunchConfiguration('enable_logging'),
                    'log_to_console': True,
                }
            ],
        ),
    ])
