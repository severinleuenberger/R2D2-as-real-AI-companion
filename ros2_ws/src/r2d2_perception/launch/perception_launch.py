#!/usr/bin/env python3
"""
Launch file for the R2D2 perception pipeline.
Starts the image_listener node to subscribe to camera frames and log diagnostics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Generate the launch description for the perception node.
    
    Returns:
        LaunchDescription: Configuration to launch the image_listener node
    """
    
    # Declare launch arguments
    debug_frame_path_arg = DeclareLaunchArgument(
        'debug_frame_path',
        default_value='/home/severin/dev/r2d2/tests/camera/perception_debug.jpg',
        description='Path to save debug frame JPEG'
    )
    
    # Create the image listener node
    image_listener_node = Node(
        package='r2d2_perception',
        executable='image_listener',
        name='image_listener',
        parameters=[
            {'debug_frame_path': LaunchConfiguration('debug_frame_path')}
        ],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        debug_frame_path_arg,
        image_listener_node,
    ])
