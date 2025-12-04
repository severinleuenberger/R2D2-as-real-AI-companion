#!/usr/bin/env python3
"""
R2D2 Perception Node - OAK-D RGB Camera Subscriber
====================================================

This node implements a basic perception pipeline that:
1. Subscribes to /oak/rgb/image_raw (published by r2d2_camera node)
2. Counts received frames
3. Calculates and logs FPS (frames per second)
4. Logs image dimensions (width, height)
5. Optionally saves one debug frame to disk

Used as a "touch-the-ground" verification that the perception pipeline
is working correctly before adding more complex processing.

Author: R2D2 Project
Date: December 2025
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from sensor_msgs.msg import Image  # ROS Image message type
import cv2  # OpenCV for image processing
from cv_bridge import CvBridge  # Convert ROS Image ↔ OpenCV mat
import time  # For FPS timing calculations
import os  # For file path operations


class PerceptionNode(Node):
    """
    ROS 2 Node for basic perception diagnostics.
    
    Subscribes to camera topic and logs:
    - Frame counter (total frames received)
    - FPS (calculated every ~1 second)
    - Image dimensions
    - Debug frame (optional)
    """

    def __init__(self):
        """Initialize the perception node with subscriptions and parameters."""
        # Call parent Node constructor with unique node name
        super().__init__('perception_node')
        
        # Frame counter: tracks total images received
        self.frame_count = 0
        
        # FPS tracking: uses timestamps from callback invocations
        self.last_log_time = time.time()  # Last time we printed FPS
        self.fps_sample_count = 0  # Frames since last FPS log
        
        # Image dimensions: extracted from first frame
        self.image_width = 0
        self.image_height = 0
        self.dimensions_logged = False  # Log dimensions only once
        
        # Debug frame handling
        self.debug_frame_saved = False  # Save only first frame
        
        # Declare ROS 2 parameters (can be overridden from command line or launch file)
        self.declare_parameter('save_debug_frame', False)
        self.declare_parameter('debug_frame_path', '/home/severin/dev/r2d2/debug_frame.jpg')
        
        # Get parameter values
        self.save_debug = self.get_parameter('save_debug_frame').value
        self.debug_path = self.get_parameter('debug_frame_path').value
        
        # Initialize cv_bridge for ROS Image ↔ OpenCV conversion
        self.bridge = CvBridge()
        
        # Create subscription to camera topic
        # Arguments:
        #   Image - message type
        #   '/oak/rgb/image_raw' - topic name
        #   self.image_callback - callback function
        #   10 - quality of service (queue size)
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(
            f'[PerceptionNode] initialized. '
            f'Subscribing to /oak/rgb/image_raw. '
            f'Debug frame saving: {self.save_debug}'
        )

    def image_callback(self, msg: Image):
        """
        Callback function invoked every time a frame arrives.
        
        This function:
        1. Increments frame counter
        2. Extracts image metadata (dimensions)
        3. Calculates FPS
        4. Saves debug frame (optional, first frame only)
        
        Args:
            msg (sensor_msgs.msg.Image): ROS Image message from camera
        """
        # Increment frame counter
        self.frame_count += 1
        self.fps_sample_count += 1
        
        # Extract image dimensions from message header
        # width and height come directly from the sensor_msgs/Image structure
        self.image_width = msg.width
        self.image_height = msg.height
        
        # Log dimensions once (on first frame)
        if not self.dimensions_logged:
            self.get_logger().info(
                f'[PerceptionNode] Image dimensions: {self.image_width}x{self.image_height}'
            )
            self.dimensions_logged = True
        
        # Calculate FPS: log metrics every ~1 second
        current_time = time.time()
        elapsed_time = current_time - self.last_log_time
        
        # If at least 1 second has passed, print FPS metrics
        if elapsed_time >= 1.0:
            fps = self.fps_sample_count / elapsed_time
            self.get_logger().info(
                f'[PerceptionNode] Frame #{self.frame_count}: '
                f'{self.image_width}x{self.image_height}, '
                f'FPS: {fps:.2f}'
            )
            
            # Reset FPS counters for next second
            self.last_log_time = current_time
            self.fps_sample_count = 0
        
        # Save debug frame (first frame only, if enabled)
        if self.save_debug and not self.debug_frame_saved:
            try:
                # Convert ROS Image message to OpenCV format (BGR)
                # cv_bridge handles the bgr8 encoding automatically
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # Create directory if it doesn't exist
                debug_dir = os.path.dirname(self.debug_path)
                if debug_dir and not os.path.exists(debug_dir):
                    os.makedirs(debug_dir, exist_ok=True)
                
                # Write JPEG file to disk
                cv2.imwrite(self.debug_path, cv_image)
                
                self.get_logger().info(
                    f'[PerceptionNode] Debug frame saved to: {self.debug_path}'
                )
                self.debug_frame_saved = True
                
            except Exception as e:
                # Log error if frame saving fails
                self.get_logger().error(
                    f'[PerceptionNode] Failed to save debug frame: {str(e)}'
                )


def main(args=None):
    """
    Main entry point for the perception node.
    
    This function:
    1. Initializes ROS 2
    2. Creates the PerceptionNode
    3. Spins (processes callbacks) until interrupted
    4. Cleans up on shutdown
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    """
    # Initialize ROS 2 client library
    rclpy.init(args=args)
    
    # Create the perception node
    node = PerceptionNode()
    
    try:
        # Spin the node: process callbacks indefinitely
        # This blocks until Ctrl+C or other shutdown signal
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Cleanup: destroy the node and shutdown ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
