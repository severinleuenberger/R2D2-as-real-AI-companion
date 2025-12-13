#!/usr/bin/env python3
"""
Launch file for R2D2 camera stream mode
Starts camera node + stream node together (for standalone streaming)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch camera + stream nodes together for standalone streaming mode
    """
    
    # Camera node (publishes /oak/rgb/image_raw)
    camera_node = Node(
        package='r2d2_camera',
        executable='camera_node',
        name='oak_d_camera',
        output='screen',
        parameters=[
            {'camera_model': 'OAK-D-LITE'},
            {'resolution_height': 1080},
            {'resolution_width': 1920},
            {'fps': 30},
        ]
    )
    
    # Camera stream node (subscribes to /oak/rgb/image_raw, serves MJPEG)
    stream_node = Node(
        package='r2d2_camera',
        executable='camera_stream_node',
        name='camera_stream_node',
        output='screen',
        parameters=[
            {'port': 8081},
            {'fps': 15},
            {'quality': 85},
            {'max_width': 1280},
            {'max_height': 720},
        ]
    )
    
    return LaunchDescription([
        camera_node,
        stream_node,
    ])

