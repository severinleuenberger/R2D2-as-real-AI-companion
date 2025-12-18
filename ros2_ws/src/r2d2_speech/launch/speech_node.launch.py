#!/usr/bin/env python3
"""R2D2 Speech Node Launch File"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate launch description"""
    
    # Get home directory
    home_dir = os.path.expanduser('~')
    r2d2_path = os.path.join(home_dir, 'dev', 'r2d2')
    venv_path = os.path.join(home_dir, 'dev', 'r2d2', 'r2d2_speech_env')
    
    # Set PYTHONPATH to include existing r2d2_speech package AND virtualenv
    python_path = f"{r2d2_path}:{venv_path}/lib/python3.10/site-packages:{os.environ.get('PYTHONPATH', '')}"
    set_pythonpath = SetEnvironmentVariable('PYTHONPATH', python_path)
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='Auto-start session on activation (false for gesture control)')
    
    instructions_arg = DeclareLaunchArgument(
        'instructions', default_value='You are the R2D2 robot from the Star Wars Movie. Speak with a slightly synthetic, system-like delivery. Use short, precise sentences. Fast-paced, efficient cadence. Recognize emotions internally, but keep vocal emotional inflection minimal. Clear, clipped articulation. Avoid unnecessary pauses. Sound efficient and machine-like.',
        description='System instructions')
    
    mic_device_arg = DeclareLaunchArgument(
        'mic_device', default_value='',
        description='Microphone device')
    
    config_file = PathJoinSubstitution([
        FindPackageShare('r2d2_speech'), 'config', 'speech_params.yaml'])
    
    speech_node = LifecycleNode(
        package='r2d2_speech',
        executable='speech_node',
        name='speech_node',
        namespace='',
        parameters=[
            config_file,
            {
                'auto_start': LaunchConfiguration('auto_start'),
                'instructions': LaunchConfiguration('instructions'),
                'mic_device': LaunchConfiguration('mic_device'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(speech_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=speech_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(speech_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    return LaunchDescription([
        set_pythonpath,
        auto_start_arg,
        instructions_arg,
        mic_device_arg,
        speech_node,
        configure_event,
        activate_event_handler,
    ])

