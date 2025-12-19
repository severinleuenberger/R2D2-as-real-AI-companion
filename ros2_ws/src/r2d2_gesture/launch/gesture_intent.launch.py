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
    auto_shutdown_timeout_seconds: float (default: 35.0 = 35 seconds)
    auto_restart_on_return: bool (default: false)
    audio_feedback_enabled: bool (default: true)
    audio_volume: float (default: 0.30)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for gesture intent node."""
    
    # Load centralized audio config file (shared with audio_notification_node)
    config_file = PathJoinSubstitution([
        FindPackageShare('r2d2_audio'), 'config', 'audio_params.yaml'])
    
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
        default_value='35.0',
        description='Seconds before auto-shutdown (default: 35 seconds)'
    )
    
    auto_restart_arg = DeclareLaunchArgument(
        'auto_restart_on_return',
        default_value='false',
        description='Auto-restart speech when person returns'
    )
    
    audio_feedback_arg = DeclareLaunchArgument(
        'audio_feedback_enabled',
        default_value='true',
        description='Enable R2D2 beep sounds for start/stop feedback'
    )
    
    audio_volume_arg = DeclareLaunchArgument(
        'audio_volume',
        default_value='0.30',
        description='Audio volume for gesture beeps (0.0-1.0) - default from config/audio_params.yaml'
    )
    
    vad_silence_timeout_arg = DeclareLaunchArgument(
        'vad_silence_timeout_seconds',
        default_value='60.0',
        description='VAD-based silence timeout (Option 2: VAD-only approach, default: 60 seconds)'
    )
    
    speaking_start_grace_arg = DeclareLaunchArgument(
        'speaking_start_grace_seconds',
        default_value='5.0',
        description='Grace period after starting conversation to ignore fist gestures (prevents false positives)'
    )
    
    # Create gesture intent node
    gesture_intent_node = Node(
        package='r2d2_gesture',
        executable='gesture_intent_node',
        name='gesture_intent_node',
        output='screen',
        parameters=[
            config_file,  # Load centralized config (audio_volume)
            {
                'cooldown_start_seconds': LaunchConfiguration('cooldown_start_seconds'),
                'cooldown_stop_seconds': LaunchConfiguration('cooldown_stop_seconds'),
                'enabled': LaunchConfiguration('enabled'),
                'auto_shutdown_enabled': LaunchConfiguration('auto_shutdown_enabled'),
                'auto_shutdown_timeout_seconds': LaunchConfiguration('auto_shutdown_timeout_seconds'),
                'auto_restart_on_return': LaunchConfiguration('auto_restart_on_return'),
                'audio_feedback_enabled': LaunchConfiguration('audio_feedback_enabled'),
                'audio_volume': LaunchConfiguration('audio_volume'),  # Can override config file
                'vad_silence_timeout_seconds': LaunchConfiguration('vad_silence_timeout_seconds'),
                'speaking_start_grace_seconds': LaunchConfiguration('speaking_start_grace_seconds'),
            }
        ]
    )
    
    return LaunchDescription([
        cooldown_start_arg,
        cooldown_stop_arg,
        enabled_arg,
        auto_shutdown_enabled_arg,
        auto_shutdown_timeout_arg,
        auto_restart_arg,
        audio_feedback_arg,
        audio_volume_arg,
        vad_silence_timeout_arg,
        speaking_start_grace_arg,
        gesture_intent_node,
    ])

