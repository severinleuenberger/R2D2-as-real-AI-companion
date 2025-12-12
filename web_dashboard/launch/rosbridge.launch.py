#!/usr/bin/env python3
"""
Launch file for rosbridge_server to enable WebSocket connection for web dashboard.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for rosbridge_server"""
    
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',  # Listen on all interfaces
            'rosbridge_compression': 'none',
            'rosbridge_encoding': 'json'
        }],
        output='screen'
    )
    
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )
    
    return LaunchDescription([
        rosbridge_node,
        rosapi_node
    ])

