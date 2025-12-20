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
            'simulate_gpio',
            default_value='false',
            description='Simulate GPIO if no hardware available'
        ),
        DeclareLaunchArgument(
            'led_mode',
            default_value='white',
            description='LED mode: white (single LED on/off) or rgb (separate color control)'
        ),
        DeclareLaunchArgument(
            'led_pin_white',
            default_value='17',
            description='GPIO pin for white LED (default: 17 = Physical Pin 22)'
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
        
        # Status LED Controller
        Node(
            package='r2d2_audio',
            executable='status_led_node',
            name='status_led_controller',
            output='screen',
            parameters=[
                {
                    'led_mode': LaunchConfiguration('led_mode'),
                    'led_pin_white': LaunchConfiguration('led_pin_white'),
                    'led_pin_red': 17,      # Backward compatibility for RGB mode
                    'led_pin_green': 27,    # Backward compatibility for RGB mode
                    'led_pin_blue': 22,     # Backward compatibility for RGB mode
                    'brightness': 1.0,
                    'update_rate_hz': 10,
                    'enabled': LaunchConfiguration('enable_led'),
                    'simulate_gpio': LaunchConfiguration('simulate_gpio'),
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
