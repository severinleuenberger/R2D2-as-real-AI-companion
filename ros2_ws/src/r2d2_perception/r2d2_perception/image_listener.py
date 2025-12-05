#!/usr/bin/env python3
"""
Image listener node for R2D2 perception pipeline.
Subscribes to OAK-D RGB camera topic and logs diagnostics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os


class ImageListener(Node):
    """
    ROS2 node that listens to camera frames and logs diagnostics.
    
    Functionality:
    - Subscribes to /oak/rgb/image_raw (sensor_msgs/msg/Image)
    - Counts incoming frames
    - Computes FPS over 1-second windows
    - Logs frame dimensions (width, height)
    - Saves one debug frame to disk
    """
    
    def __init__(self):
        """Initialize the image listener node with subscriptions and counters."""
        super().__init__('image_listener')
        
        # Initialize ROS2 parameters
        self.declare_parameter('debug_frame_path', '/home/severin/dev/r2d2/tests/camera/perception_debug.jpg')
        self.debug_frame_path = self.get_parameter('debug_frame_path').value
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Initialize frame tracking
        self.frame_count = 0
        self.last_log_time = time.time()
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        
        # Debug frame flag - save only once
        self.debug_frame_saved = False
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info('ImageListener node initialized, subscribed to /oak/rgb/image_raw')
    
    def image_callback(self, msg: Image):
        """
        Callback function triggered when a new frame arrives.
        
        Args:
            msg (Image): ROS2 Image message from camera
        """
        # Increment frame counter
        self.frame_count += 1
        self.fps_frame_count += 1
        
        # Extract image dimensions from message metadata
        width = msg.width
        height = msg.height
        
        # Log frame information every ~1 second
        current_time = time.time()
        elapsed = current_time - self.last_log_time
        
        if elapsed >= 1.0:
            # Calculate FPS for this 1-second window
            fps = self.fps_frame_count / elapsed
            self.get_logger().info(
                f'Frame #{self.frame_count} | FPS: {fps:.2f} | '
                f'Dimensions: {width}x{height}'
            )
            
            # Reset counters for next window
            self.last_log_time = current_time
            self.fps_frame_count = 0
        
        # Save debug frame only once
        if not self.debug_frame_saved:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # Create directory if it doesn't exist
                debug_dir = os.path.dirname(self.debug_frame_path)
                os.makedirs(debug_dir, exist_ok=True)
                
                # Save frame as JPEG
                cv2.imwrite(self.debug_frame_path, cv_image)
                self.get_logger().info(f'Debug frame saved to: {self.debug_frame_path}')
                
                # Mark that we've saved the debug frame (no more saves)
                self.debug_frame_saved = True
                
            except Exception as e:
                self.get_logger().error(f'Failed to save debug frame: {e}')


def main(args=None):
    """
    Main entry point for the image listener node.
    Initializes ROS2 and spins the node.
    """
    rclpy.init(args=args)
    
    # Create and spin the node
    image_listener = ImageListener()
    rclpy.spin(image_listener)
    
    # Cleanup on shutdown
    image_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
