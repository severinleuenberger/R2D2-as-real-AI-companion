#!/usr/bin/env python3
"""
Image listener and perception node for R2D2 perception pipeline.
Subscribes to OAK-D RGB camera topic, performs image processing, and publishes perception metrics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np


class ImageListener(Node):
    """
    ROS2 perception node that processes camera frames and publishes perception metrics.
    
    Functionality:
    - Subscribes to /oak/rgb/image_raw (sensor_msgs/msg/Image)
    - Counts incoming frames and tracks FPS
    - Extracts image dimensions (width, height)
    - Performs image processing:
      * Downscales image to 640x360
      * Converts to grayscale
      * Computes mean brightness
    - Detects faces using OpenCV Haar Cascade
    - Publishes brightness on /r2d2/perception/brightness (std_msgs/Float32)
    - Publishes face count on /r2d2/perception/face_count (std_msgs/Int32)
    - Saves debug frames (RGB and grayscale) on demand
    """
    
    def __init__(self):
        """Initialize the perception node with subscriptions, publishers, and parameters."""
        super().__init__('image_listener')
        
        # Declare ROS 2 parameters
        self.declare_parameter('debug_frame_path', '/home/severin/dev/r2d2/tests/camera/perception_debug.jpg')
        self.declare_parameter('save_debug_gray_frame', False)
        self.declare_parameter('debug_gray_frame_path', '/home/severin/dev/r2d2/tests/camera/perception_debug_gray.jpg')
        self.declare_parameter('log_every_n_frames', 30)  # Log brightness every N frames
        self.declare_parameter('log_face_detections', False)  # Enable verbose face detection logging
        
        # Get parameter values
        self.debug_frame_path = self.get_parameter('debug_frame_path').value
        self.save_debug_gray = self.get_parameter('save_debug_gray_frame').value
        self.debug_gray_frame_path = self.get_parameter('debug_gray_frame_path').value
        self.log_every_n = self.get_parameter('log_every_n_frames').value
        self.log_faces = self.get_parameter('log_face_detections').value
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for brightness metric
        self.brightness_publisher = self.create_publisher(
            Float32,
            '/r2d2/perception/brightness',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create publisher for face count metric
        self.face_count_publisher = self.create_publisher(
            Int32,
            '/r2d2/perception/face_count',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Initialize frame tracking
        self.frame_count = 0
        self.last_log_time = time.time()
        self.fps_frame_count = 0
        
        # Debug frame flags - save only once for each
        self.debug_rgb_saved = False
        self.debug_gray_saved = False
        
        # CV bridge for ROS Image ↔ OpenCV conversion
        self.bridge = CvBridge()
        
        # Load face cascade classifier for face detection
        # Try multiple paths for compatibility with different OpenCV installations
        cascade_paths = [
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'  if hasattr(cv2, 'data') else '',
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        ]
        
        self.face_cascade = None
        cascade_found = False
        for cascade_path in cascade_paths:
            if not cascade_path or not os.path.exists(cascade_path):
                continue
            try:
                self.face_cascade = cv2.CascadeClassifier(cascade_path)
                if not self.face_cascade.empty():
                    cascade_found = True
                    self.get_logger().info(f'Haar Cascade loaded successfully from {cascade_path}')
                    break
            except Exception:
                continue
        
        if not cascade_found or self.face_cascade is None or self.face_cascade.empty():
            self.face_cascade = None
            self.get_logger().warn('Failed to load Haar Cascade. Face detection will be disabled.')
        
        # Face detection parameters
        self.cascade_scale_factor = 1.05
        self.cascade_min_neighbors = 5
        self.cascade_min_size = (30, 30)
        self.cascade_max_size = (500, 500)
        
        self.get_logger().info('ImageListener node initialized, subscribed to /oak/rgb/image_raw')
        self.get_logger().info(f'Face detection enabled with Haar Cascade classifier')
    
    def image_callback(self, msg: Image):
        """
        Callback function triggered when a new frame arrives.
        Processes the image and publishes perception metrics.
        
        Args:
            msg (Image): ROS2 Image message from camera
        """
        # Increment frame counter
        self.frame_count += 1
        self.fps_frame_count += 1
        
        # Extract original image dimensions from message metadata
        original_width = msg.width
        original_height = msg.height
        
        # Convert ROS Image message to OpenCV BGR format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Downscale image to 640x360 for faster processing
        downscaled = cv2.resize(cv_image, (640, 360))
        
        # Convert downscaled image to grayscale
        gray_image = cv2.cvtColor(downscaled, cv2.COLOR_BGR2GRAY)
        
        # Compute mean brightness from grayscale image (0-255 range)
        mean_brightness = float(np.mean(gray_image))
        
        # Publish brightness value on perception topic
        brightness_msg = Float32()
        brightness_msg.data = mean_brightness
        self.brightness_publisher.publish(brightness_msg)
        
        # Detect faces using Haar Cascade on grayscale image
        face_count = 0
        faces = []
        if self.face_cascade is not None and not self.face_cascade.empty():
            try:
                faces = self.face_cascade.detectMultiScale(
                    gray_image,
                    scaleFactor=self.cascade_scale_factor,
                    minNeighbors=self.cascade_min_neighbors,
                    minSize=self.cascade_min_size,
                    maxSize=self.cascade_max_size,
                    flags=cv2.CASCADE_SCALE_IMAGE
                )
                face_count = len(faces)
            except Exception as e:
                self.get_logger().error(f'Face detection failed: {e}')
        
        # Publish face count
        face_count_msg = Int32()
        face_count_msg.data = face_count
        self.face_count_publisher.publish(face_count_msg)
        
        # Log every N frames to keep output readable
        if self.frame_count % self.log_every_n == 0:
            current_time = time.time()
            elapsed = current_time - self.last_log_time
            
            if elapsed >= 1.0:
                # Calculate FPS for this time window
                fps = self.fps_frame_count / elapsed
                self.get_logger().info(
                    f'Frame #{self.frame_count} | FPS: {fps:.2f} | '
                    f'Original: {original_width}x{original_height} | '
                    f'Brightness: {mean_brightness:.1f} | Faces: {face_count}'
                )
                
                # Log bounding box details if verbose face detection is enabled
                if self.log_faces and face_count > 0:
                    for i, (x, y, w, h) in enumerate(faces):
                        self.get_logger().info(
                            f'  Face {i+1}: position=({x}, {y}), size={w}x{h}'
                        )
                
                # Reset counters for next window
                self.last_log_time = current_time
                self.fps_frame_count = 0
        
        # Save RGB debug frame once (first frame)
        if not self.debug_rgb_saved:
            try:
                debug_dir = os.path.dirname(self.debug_frame_path)
                os.makedirs(debug_dir, exist_ok=True)
                cv2.imwrite(self.debug_frame_path, cv_image)
                self.get_logger().info(f'RGB debug frame saved to: {self.debug_frame_path}')
                self.debug_rgb_saved = True
            except Exception as e:
                self.get_logger().error(f'Failed to save RGB debug frame: {e}')
        
        # Save grayscale debug frame once if enabled
        if self.save_debug_gray and not self.debug_gray_saved:
            try:
                debug_dir = os.path.dirname(self.debug_gray_frame_path)
                os.makedirs(debug_dir, exist_ok=True)
                cv2.imwrite(self.debug_gray_frame_path, gray_image)
                self.get_logger().info(f'Grayscale debug frame saved to: {self.debug_gray_frame_path}')
                self.debug_gray_saved = True
            except Exception as e:
                self.get_logger().error(f'Failed to save grayscale debug frame: {e}')


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
