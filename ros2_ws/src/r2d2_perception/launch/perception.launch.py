#!/usr/bin/env python3
"""
Launch file for the R2D2 perception pipeline.
Starts the image_listener node to perform image processing and publish perception metrics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Generate the launch description for the perception node.
    
    Returns:
        LaunchDescription: Configuration to launch the image_listener node with parameters
    """
    
    # Declare launch arguments for RGB debug frame
    debug_frame_path_arg = DeclareLaunchArgument(
        'debug_frame_path',
        default_value='/home/severin/dev/r2d2/tests/camera/perception_debug.jpg',
        description='Path to save RGB debug frame JPEG'
    )
    
    # Declare launch argument for grayscale debug frame saving
    save_debug_gray_arg = DeclareLaunchArgument(
        'save_debug_gray_frame',
        default_value='false',
        description='Enable saving grayscale debug frame'
    )
    
    # Declare launch argument for grayscale debug frame path
    debug_gray_path_arg = DeclareLaunchArgument(
        'debug_gray_frame_path',
        default_value='/home/severin/dev/r2d2/tests/camera/perception_debug_gray.jpg',
        description='Path to save grayscale debug frame JPEG'
    )
    
    # Declare launch argument for logging frequency
    log_every_n_arg = DeclareLaunchArgument(
        'log_every_n_frames',
        default_value='30',
        description='Log perception metrics every N frames'
    )
    
    # Create the image listener node with all parameters
    image_listener_node = Node(
        package='r2d2_perception',
        executable='image_listener',
        name='image_listener',
        parameters=[
            {'debug_frame_path': LaunchConfiguration('debug_frame_path')},
            {'save_debug_gray_frame': LaunchConfiguration('save_debug_gray_frame')},
            {'debug_gray_frame_path': LaunchConfiguration('debug_gray_frame_path')},
            {'log_every_n_frames': LaunchConfiguration('log_every_n_frames')},
        ],
        output='screen'
    )
    
    # Return launch description with all arguments and node
    return LaunchDescription([
        debug_frame_path_arg,
        save_debug_gray_arg,
        debug_gray_path_arg,
        log_every_n_arg,
        image_listener_node,
    ])
