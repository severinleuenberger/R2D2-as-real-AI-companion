from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='r2d2_llm',
            executable='tts_node.py',
            name='tts_node',
            output='screen'
        ),
        Node(
            package='r2d2_llm',
            executable='stt_node.py',
            name='stt_node',
            output='screen'
        ),
    ])
