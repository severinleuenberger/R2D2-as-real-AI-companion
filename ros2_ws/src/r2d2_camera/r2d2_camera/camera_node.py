#!/usr/bin/env python3
"""
OAK-D Lite Camera ROS 2 Node for R2D2
Publishes RGB frames as sensor_msgs/Image to /oak/rgb/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import threading


class OAKDCameraNode(Node):
    def __init__(self):
        super().__init__('oak_d_camera')
        
        # Declare parameters
        self.declare_parameter('camera_model', 'OAK-D')
        self.declare_parameter('resolution_height', 1080)
        self.declare_parameter('resolution_width', 1920)
        self.declare_parameter('fps', 30)
        
        self.camera_model = self.get_parameter('camera_model').value
        self.height = self.get_parameter('resolution_height').value
        self.width = self.get_parameter('resolution_width').value
        self.fps = self.get_parameter('fps').value
        
        # Create publisher
        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        
        # CV Bridge for OpenCV to ROS conversion
        self.bridge = CvBridge()
        
        # Frame counter and diagnostics
        self.frame_count = 0
        self.is_running = True
        
        self.get_logger().info(f"Initializing {self.camera_model} camera")
        self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.fps} FPS")
        
        # Initialize camera in separate thread
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
    
    def _camera_loop(self):
        """Main camera capture loop"""
        device = None
        try:
            # Create DepthAI pipeline
            pipeline = dai.Pipeline()
            
            # Create RGB camera node
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(self.fps)
            cam_rgb.setInterleaved(False)
            
            # Create XLink output
            xout_rgb = pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName("rgb")
            cam_rgb.video.link(xout_rgb.input)
            
            # Start device with proper error handling
            device = dai.Device(pipeline)
            q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            self.get_logger().info("Camera device initialized successfully")
            
            while self.is_running and device is not None:
                in_rgb = q.get()
                if in_rgb is None:
                    continue
                
                frame = in_rgb.getCvFrame()
                self.frame_count += 1
                
                # Convert to ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.frame_id = "oak_d_rgb_camera_optical_frame"
                img_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Publish
                self.publisher.publish(img_msg)
                
                if self.frame_count % 30 == 0:
                    self.get_logger().debug(f"Published {self.frame_count} frames")
        
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
            self.is_running = False
        finally:
            if device is not None:
                del device
    
    def destroy_node(self):
        self.is_running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = OAKDCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
