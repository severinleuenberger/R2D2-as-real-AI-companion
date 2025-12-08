#!/usr/bin/env python3
"""
Launch configuration for R2D2 Audio Notification Node - ENHANCED

Enhanced with improved state tracking:
- Tolerates brief recognition gaps (< 5 seconds)
- Confirms loss after > 5 seconds of continuous absence
- Single beep on recognition, double beep on loss

Usage:
  # Default settings (recognize "severin", beep at 1000Hz)
  ros2 launch r2d2_audio audio_notification.launch.py

  # Custom settings with loss detection
  ros2 launch r2d2_audio audio_notification.launch.py \
    target_person:=severin \
    beep_frequency:=1200 \
    loss_beep_frequency:=600 \
    jitter_tolerance_seconds:=5.0 \
    loss_confirmation_seconds:=5.0 \
    enabled:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for enhanced audio notification node."""
    
    return LaunchDescription([
        # Target person
        DeclareLaunchArgument(
            'target_person',
            default_value='severin',
            description='Name of person to recognize'
        ),
        
        # Recognition beep parameters
        DeclareLaunchArgument(
            'beep_frequency',
            default_value='400.0',
            description='Recognition beep frequency in Hz (20-20000)'
        ),
        DeclareLaunchArgument(
            'beep_duration',
            default_value='0.5',
            description='Recognition beep duration in seconds (0.1-10.0)'
        ),
        DeclareLaunchArgument(
            'beep_volume',
            default_value='0.25',
            description='Beep volume 0.0-1.0 (0=silent, 1=max)'
        ),
        
        # Loss notification beep parameters
        DeclareLaunchArgument(
            'loss_beep_frequency',
            default_value='400.0',
            description='Loss alert beep frequency in Hz (20-20000)'
        ),
        DeclareLaunchArgument(
            'loss_beep_duration',
            default_value='0.3',
            description='Loss alert beep duration per beep in seconds'
        ),
        
        # Jitter tolerance (brief gap tolerance)
        DeclareLaunchArgument(
            'jitter_tolerance_seconds',
            default_value='5.0',
            description='Tolerance for brief recognition gaps (keeps status "recognized" during brief losses)'
        ),
        
        # Loss confirmation (time before confirming loss)
        DeclareLaunchArgument(
            'loss_confirmation_seconds',
            default_value='5.0',
            description='Time to wait before confirming loss and playing double beep'
        ),
        
        # Recognition cooldown
        DeclareLaunchArgument(
            'cooldown_seconds',
            default_value='2.0',
            description='Minimum seconds between recognition beeps'
        ),
        
        # Enable/disable
        DeclareLaunchArgument(
            'enabled',
            default_value='true',
            description='Enable/disable audio notifications'
        ),
        
        # Audio notification node
        Node(
            package='r2d2_audio',
            executable='audio_notification_node',
            name='audio_notification_node',
            output='screen',
            parameters=[{
                'target_person': LaunchConfiguration('target_person'),
                'beep_frequency': LaunchConfiguration('beep_frequency'),
                'beep_duration': LaunchConfiguration('beep_duration'),
                'beep_volume': LaunchConfiguration('beep_volume'),
                'loss_beep_frequency': LaunchConfiguration('loss_beep_frequency'),
                'loss_beep_duration': LaunchConfiguration('loss_beep_duration'),
                'jitter_tolerance_seconds': LaunchConfiguration('jitter_tolerance_seconds'),
                'loss_confirmation_seconds': LaunchConfiguration('loss_confirmation_seconds'),
                'cooldown_seconds': LaunchConfiguration('cooldown_seconds'),
                'enabled': LaunchConfiguration('enabled'),
            }]
        ),
    ])
