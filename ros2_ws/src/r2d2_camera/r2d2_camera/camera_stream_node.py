#!/usr/bin/env python3
"""
Camera Stream Node for R2D2
Subscribes to /oak/rgb/image_raw and serves MJPEG stream via HTTP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import http.server
import socketserver
from io import BytesIO
import time

def make_mjpeg_handler(frame_buffer):
    """Factory function to create MJPEG handler with frame buffer"""
    class MJPEGHandler(http.server.BaseHTTPRequestHandler):
        """HTTP handler for MJPEG streaming"""
        
        def do_GET(self):
            if self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'close')
                self.end_headers()
                
                try:
                    while True:
                        frame_data = frame_buffer.get_frame()
                        if frame_data:
                            self.wfile.write(b'--jpgboundary\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(frame_data)))
                            self.end_headers()
                            self.wfile.write(frame_data)
                            self.wfile.write(b'\r\n')
                        else:
                            time.sleep(0.1)
                except (ConnectionResetError, BrokenPipeError):
                    pass
            elif self.path == '/health':
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK')
            else:
                self.send_response(404)
                self.end_headers()
        
        def log_message(self, format, *args):
            # Suppress default logging
            pass
    
    return MJPEGHandler


class FrameBuffer:
    """Thread-safe frame buffer"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.frame_data = None
        self.last_update = 0
    
    def update_frame(self, frame_data):
        with self.lock:
            self.frame_data = frame_data
            self.last_update = time.time()
    
    def get_frame(self):
        with self.lock:
            return self.frame_data


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        
        # Declare parameters
        self.declare_parameter('port', 8081)
        self.declare_parameter('fps', 15)
        self.declare_parameter('quality', 85)
        self.declare_parameter('max_width', 1280)
        self.declare_parameter('max_height', 720)
        
        self.port = self.get_parameter('port').value
        self.target_fps = self.get_parameter('fps').value
        self.quality = self.get_parameter('quality').value
        self.max_width = self.get_parameter('max_width').value
        self.max_height = self.get_parameter('max_height').value
        
        # CV Bridge for ROS to OpenCV conversion
        self.bridge = CvBridge()
        
        # Frame buffer for HTTP streaming
        self.frame_buffer = FrameBuffer()
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # Frame rate control
        self.frame_interval = 1.0 / self.target_fps
        self.last_frame_time = 0
        
        self.get_logger().info(f'Camera stream node initialized')
        self.get_logger().info(f'Streaming on port {self.port} at {self.target_fps} FPS')
        self.get_logger().info(f'Max resolution: {self.max_width}x{self.max_height}')
        
        # Start HTTP server in separate thread
        self.server_thread = threading.Thread(target=self._start_server, daemon=True)
        self.server_thread.start()
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        current_time = time.time()
        
        # Frame rate limiting
        if current_time - self.last_frame_time < self.frame_interval:
            return
        
        self.last_frame_time = current_time
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            height, width = cv_image.shape[:2]
            if width > self.max_width or height > self.max_height:
                scale = min(self.max_width / width, self.max_height / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            
            # Encode to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            result, jpeg_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            if result:
                # Update frame buffer
                self.frame_buffer.update_frame(jpeg_data.tobytes())
        
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    def _start_server(self):
        """Start HTTP server for MJPEG streaming"""
        handler_class = make_mjpeg_handler(self.frame_buffer)
        
        try:
            with socketserver.TCPServer(("0.0.0.0", self.port), handler_class) as httpd:
                self.get_logger().info(f'MJPEG stream server started on port {self.port}')
                httpd.serve_forever()
        except OSError as e:
            self.get_logger().error(f'Failed to start HTTP server: {e}')
        except Exception as e:
            self.get_logger().error(f'HTTP server error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera stream node...')
    except Exception as e:
        node.get_logger().error(f'Error in camera stream node: {e}')
    finally:
        try:
            if node:
                node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

