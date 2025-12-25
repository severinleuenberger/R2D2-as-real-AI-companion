#!/usr/bin/env python3
"""R2D2 REST Speech Node (Intelligent Mode) Launch File"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate launch description for REST Speech Node (Intelligent Mode)"""
    
    # Get home directory
    home_dir = os.path.expanduser('~')
    r2d2_path = os.path.join(home_dir, 'dev', 'r2d2')
    venv_path = os.path.join(home_dir, 'dev', 'r2d2', 'r2d2_speech_env')
    
    # Set PYTHONPATH to include existing r2d2_speech package AND virtualenv
    python_path = f"{r2d2_path}:{venv_path}/lib/python3.10/site-packages:{os.environ.get('PYTHONPATH', '')}"
    set_pythonpath = SetEnvironmentVariable('PYTHONPATH', python_path)
    
    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('r2d2_speech'), 'config', 'speech_params.yaml'])
    
    # REST Speech Node (Intelligent Mode)
    rest_speech_node = LifecycleNode(
        package='r2d2_speech',
        executable='rest_speech_node',
        name='rest_speech_node',
        namespace='',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Auto-configure on startup
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(rest_speech_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Auto-activate after configuration
    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=rest_speech_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(rest_speech_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    return LaunchDescription([
        set_pythonpath,
        rest_speech_node,
        configure_event,
        activate_event_handler,
    ])

