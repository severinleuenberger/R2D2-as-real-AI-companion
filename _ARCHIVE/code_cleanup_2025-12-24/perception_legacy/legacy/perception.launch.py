"""
Launch file for R2D2 Perception Node
====================================

This launch file starts the perception node with optional parameters.

Usage:
    # Launch with default parameters (no debug frame saving)
    ros2 launch r2d2_perception perception.launch.py
    
    # Launch with debug frame saving enabled
    ros2 launch r2d2_perception perception.launch.py save_debug_frame:=true
    
    # Custom debug frame path
    ros2 launch r2d2_perception perception.launch.py \
        save_debug_frame:=true \
        debug_frame_path:=/tmp/perception_debug.jpg
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the perception node.
    
    This function:
    1. Declares launch arguments (parameters that can be overridden)
    2. Creates the perception_node with parameter substitutions
    3. Returns the complete launch description
    
    Returns:
        LaunchDescription: Complete ROS 2 launch configuration
    """
    
    # Declare launch argument: whether to save debug frame
    # Default: False (disabled)
    # Can be overridden with: save_debug_frame:=true
    save_debug_frame_arg = DeclareLaunchArgument(
        'save_debug_frame',
        default_value='false',
        description='Enable saving first received frame as debug image'
    )
    
    # Declare launch argument: path where debug frame is saved
    # Default: /home/severin/dev/r2d2/debug_frame.jpg
    # Can be overridden with: debug_frame_path:=/custom/path.jpg
    debug_frame_path_arg = DeclareLaunchArgument(
        'debug_frame_path',
        default_value='/home/severin/dev/r2d2/debug_frame.jpg',
        description='Absolute path where first frame will be saved (if enabled)'
    )
    
    # Create the perception node with parameter mappings
    perception_node = Node(
        package='r2d2_perception',  # Package name
        executable='perception_node',  # Entry point (from setup.py)
        name='perception_node',  # Node name in ROS 2 graph
        output='screen',  # Print logs to console
        parameters=[
            {
                'save_debug_frame': LaunchConfiguration('save_debug_frame'),
                'debug_frame_path': LaunchConfiguration('debug_frame_path'),
            }
        ],
    )
    
    # Create and return the launch description
    # This combines all launch arguments and nodes
    return LaunchDescription([
        save_debug_frame_arg,
        debug_frame_path_arg,
        perception_node,
    ])
