#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            'say',
            self.say_callback,
            10)
        self.get_logger().info('TTS Node started - say something on /say topic!')

    def say_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Saying: {text}')
        # Use espeak for robot voice (install: sudo apt install espeak-ng)
        subprocess.run(['espeak-ng', '-v', 'en-us', '-s', '150', text])

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
