import rclpy
from rclpy.node import Node

class BeepNode(Node):
    def __init__(self):
        super().__init__('r2d2_beep_node')
        self.get_logger().info('BeepNode gestartet 🚀')
        # alle 2 Sekunden loggen
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Beep! R2D2 is alive.')

def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
