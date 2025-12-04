import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('r2d2_heartbeat_node')
        self.publisher = self.create_publisher(String, '/r2d2/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('HeartbeatNode gestartet ❤️')

    def timer_callback(self):
        msg = String()
        msg.data = f'R2D2 heartbeat @ {datetime.now().isoformat(timespec="seconds")}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
