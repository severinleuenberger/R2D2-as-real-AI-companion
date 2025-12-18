#!/usr/bin/env python3
"""
Launch configuration for R2D2 Audio Notification Node - AUDIO FILES

Uses MP3 audio files instead of generated beeps.

Audio Files:
- Recognition: Voicy_R2-D2 - 2.mp3
- Loss Alert: Voicy_R2-D2 - 5.mp3

Usage:
  # Default settings (recognize target person, volume 5%)
  ros2 launch r2d2_audio audio_notification.launch.py

  # Custom settings with different volume and target person
  ros2 launch r2d2_audio audio_notification.launch.py \
    target_person:=alice \
    audio_volume:=0.5 \
    alsa_device:=hw:1,0 \
    jitter_tolerance_seconds:=5.0 \
    loss_confirmation_seconds:=10.0 \
    enabled:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for enhanced audio notification node."""
    
    # Load centralized audio config file
    config_file = PathJoinSubstitution([
        FindPackageShare('r2d2_audio'), 'config', 'audio_params.yaml'])
    
    return LaunchDescription([
        # Target person
        DeclareLaunchArgument(
            'target_person',
            default_value='target_person',
            description='Name of person to recognize (should match training data)'
        ),
        
        # Audio volume (NOTE: default comes from config/audio_params.yaml)
        DeclareLaunchArgument(
            'audio_volume',
            default_value='0.30',
            description='Audio volume 0.0-1.0 (0=silent, 0.05=very quiet, 0.30=medium, 1.0=max)'
        ),
        
        # ALSA device
        DeclareLaunchArgument(
            'alsa_device',
            default_value='hw:1,0',
            description='ALSA device for audio output (e.g., hw:1,0)'
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
            default_value='15.0',
            description='Time to wait before confirming loss and playing alert audio (GLOBAL parameter)'
        ),
        
        # Recognition cooldown
        DeclareLaunchArgument(
            'cooldown_seconds',
            default_value='2.0',
            description='Minimum seconds between recognition alerts'
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
            parameters=[
                config_file,  # Load centralized config (audio_volume)
                {
                    'target_person': LaunchConfiguration('target_person'),
                    'audio_volume': LaunchConfiguration('audio_volume'),  # Can override config file
                    'alsa_device': LaunchConfiguration('alsa_device'),
                    'jitter_tolerance_seconds': LaunchConfiguration('jitter_tolerance_seconds'),
                    'loss_confirmation_seconds': LaunchConfiguration('loss_confirmation_seconds'),
                    'cooldown_seconds': LaunchConfiguration('cooldown_seconds'),
                    'enabled': LaunchConfiguration('enabled'),
                }
            ]
        ),
    ])
