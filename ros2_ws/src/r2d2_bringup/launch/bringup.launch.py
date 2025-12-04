from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    beep = Node(
        package='r2d2_hello',
        executable='beep_node',
        name='r2d2_beep_node',
        output='screen'
    )

    heartbeat = Node(
        package='r2d2_hello',
        executable='heartbeat_node',
        name='r2d2_heartbeat_node',
        output='screen'
    )

    return LaunchDescription([
        beep,
        heartbeat
    ])
