from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the OAK-D Lite camera node
    """
    
    # Camera node
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
    
    return LaunchDescription([
        camera_node,
    ])
