#!/usr/bin/env python3
"""
Verify 2Hz Status Stream - Terminal Display
Subscribes to /r2d2/audio/person_status and displays updates at 2Hz (every 500ms)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from datetime import datetime

class StatusStreamVerifier(Node):
    """Node that subscribes to person_status and displays at 2Hz"""
    
    def __init__(self):
        super().__init__('status_stream_verifier')
        
        self.last_display_time = 0
        self.update_interval = 0.5  # 2 Hz = 500ms
        self.message_count = 0
        self.display_count = 0
        
        # Subscribe to person_status topic
        self.subscription = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info('Status Stream Verifier started')
        self.get_logger().info('Subscribed to /r2d2/audio/person_status')
        self.get_logger().info('Displaying updates at 2Hz (every 500ms)')
        self.get_logger().info('Format: status | person_identity | hh:mm:ss | confidence%')
        self.get_logger().info('Press Ctrl+C to stop\n')
        
    def status_callback(self, msg):
        """Handle incoming status messages"""
        self.message_count += 1
        
        try:
            status = json.loads(msg.data)
            
            # Throttle display to 2Hz
            current_time = time.time()
            if current_time - self.last_display_time >= self.update_interval:
                self.display_status(status)
                self.last_display_time = current_time
                self.display_count += 1
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
    
    def display_status(self, status):
        """Display status in the requested format"""
        # Format timestamp to hh:mm:ss
        timestamp_sec = status.get('timestamp_sec', 0)
        timestamp_nanosec = status.get('timestamp_nanosec', 0)
        dt = datetime.fromtimestamp(timestamp_sec + timestamp_nanosec / 1e9)
        time_str = dt.strftime('%H:%M:%S')
        
        # Get values with defaults
        status_val = status.get('status', 'unknown')
        person = status.get('person_identity', 'unknown')
        confidence = status.get('confidence', 0.0)
        confidence_pct = (confidence * 100) if isinstance(confidence, (int, float)) else 0.0
        
        # Display in format: status | person_identity | hh:mm:ss | confidence%
        print(f'{status_val:6s} | {person:15s} | {time_str} | {confidence_pct:5.1f}%')
        
        # Show stats every 10 displays
        if self.display_count % 10 == 0:
            rate = self.message_count / (time.time() - self.start_time) if hasattr(self, 'start_time') else 0
            print(f'--- Stats: {self.message_count} messages received, {self.display_count} displayed, rate: {rate:.1f} Hz ---')
    
    def run(self):
        """Run the verifier"""
        self.start_time = time.time()
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('\n\nStopping verifier...')
            elapsed = time.time() - self.start_time
            print(f'\nFinal stats:')
            print(f'  Messages received: {self.message_count}')
            print(f'  Messages displayed: {self.display_count}')
            print(f'  Total time: {elapsed:.1f}s')
            print(f'  Input rate: {self.message_count/elapsed:.1f} Hz')
            print(f'  Display rate: {self.display_count/elapsed:.1f} Hz (target: 2.0 Hz)')

def main(args=None):
    rclpy.init(args=args)
    verifier = StatusStreamVerifier()
    verifier.run()
    verifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
