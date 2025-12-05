#!/usr/bin/env python3
"""
Launch file for R2D2 camera and perception pipeline.
Starts both the OAK-D camera driver and the perception node with image processing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for camera + perception pipeline.
    
    Includes:
    - OAK-D camera driver (r2d2_camera)
    - Perception node (r2d2_perception) for image processing and brightness metrics
    """
    
    # Find package shares
    r2d2_camera_share = FindPackageShare('r2d2_camera')
    r2d2_perception_share = FindPackageShare('r2d2_perception')
    
    # Declare launch arguments for perception node
    save_debug_gray_arg = DeclareLaunchArgument(
        'save_debug_gray_frame',
        default_value='false',
        description='Enable saving grayscale debug frame'
    )
    
    log_every_n_arg = DeclareLaunchArgument(
        'log_every_n_frames',
        default_value='30',
        description='Log perception metrics every N frames'
    )
    
    # Include camera launch file from r2d2_camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([r2d2_camera_share, 'launch', 'camera.launch.py'])
        )
    )
    
    # Include perception launch file from r2d2_perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([r2d2_perception_share, 'launch', 'perception.launch.py'])
        ),
        launch_arguments={
            'save_debug_gray_frame': LaunchConfiguration('save_debug_gray_frame'),
            'log_every_n_frames': LaunchConfiguration('log_every_n_frames'),
        }.items()
    )
    
    # Return complete launch description
    return LaunchDescription([
        save_debug_gray_arg,
        log_every_n_arg,
        camera_launch,
        perception_launch,
    ])
