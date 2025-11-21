#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher = self.create_publisher(String, 'heard', 10)
        self.get_logger().info('STT Node started - type to simulate speech')
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        # Dummy input for testing (replace with real mic later)
        input_text = input("Say something (type and press Enter): ")
        msg = String()
        msg.data = input_text
        self.publisher.publish(msg)
        self.get_logger().info(f'Heard: {input_text}')

def main(args=None):
    rclpy.init(args=args)
    stt_node = STTNode()
    rclpy.spin(stt_node)
    stt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
