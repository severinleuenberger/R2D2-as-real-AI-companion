"""
R2D2 Heartbeat Node - Simple "alive" ping

Publishes a lightweight heartbeat every second to indicate the system is running.
System metrics (CPU, GPU, Disk, Temp) are now collected on-demand via the 
Web Dashboard's /api/system/health endpoint.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import json


class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('r2d2_heartbeat_node')
        self.publisher = self.create_publisher(String, '/r2d2/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('HeartbeatNode started ❤️ (lightweight mode)')

    def timer_callback(self):
        """Publish simple heartbeat - just timestamp and status"""
        heartbeat_data = {
            'timestamp': datetime.now().isoformat(timespec="seconds"),
            'status': 'running'
        }
        
        msg = String()
        msg.data = json.dumps(heartbeat_data)
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published heartbeat: {msg.data}')


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
