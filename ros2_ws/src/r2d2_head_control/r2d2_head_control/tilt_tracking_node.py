#!/usr/bin/env python3
"""
Tilt Tracking Node - Vertical face tracking using RC servo.

Subscribes to face bounding box position and adjusts camera tilt servo
to keep the detected face centered in the frame.

Features:
- Smooth tracking with PID control
- Deadband to avoid jitter for small movements
- Input smoothing (exponential filter)
- Graceful loss handling (hold, then return to neutral)
- Gating by person status (only track when face visible)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
import json
import time

from .pid_controller import PIDController
from .servo_driver import ServoDriver


class TiltTrackingNode(Node):
    """
    ROS 2 node for vertical face tracking via tilt servo.
    """
    
    def __init__(self):
        super().__init__('tilt_tracking_node')
        
        self.get_logger().info("Tilt Tracking Node initializing...")
        
        # Declare parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('tracking_rate_hz', 30.0)
        
        # Input processing
        self.declare_parameter('deadband', 0.08)  # Fraction of frame height
        self.declare_parameter('smoothing_alpha', 0.3)  # 0=heavy smoothing, 1=no smoothing
        
        # PID gains
        self.declare_parameter('kp', 30.0)  # Degrees per unit error
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 5.0)
        
        # Servo limits (degrees)
        self.declare_parameter('min_angle', 60.0)
        self.declare_parameter('max_angle', 120.0)
        self.declare_parameter('neutral_angle', 90.0)
        
        # Loss handling
        self.declare_parameter('hold_timeout_sec', 0.5)
        self.declare_parameter('neutral_timeout_sec', 3.0)
        self.declare_parameter('neutral_return_speed', 5.0)  # Degrees per second
        
        # Hardware
        self.declare_parameter('servo_gpio_pin', 13)
        self.declare_parameter('simulate_hardware', False)
        
        # Get parameter values
        self.enabled = self.get_parameter('enabled').value
        self.tracking_rate_hz = self.get_parameter('tracking_rate_hz').value
        self.deadband = self.get_parameter('deadband').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.neutral_angle = self.get_parameter('neutral_angle').value
        
        self.hold_timeout_sec = self.get_parameter('hold_timeout_sec').value
        self.neutral_timeout_sec = self.get_parameter('neutral_timeout_sec').value
        self.neutral_return_speed = self.get_parameter('neutral_return_speed').value
        
        self.servo_gpio_pin = self.get_parameter('servo_gpio_pin').value
        self.simulate_hardware = self.get_parameter('simulate_hardware').value
        
        # Initialize state
        self.current_status = "blue"  # RED, GREEN, BLUE from person_status
        self.smoothed_y = 0.5  # Smoothed face Y position (0.5 = center)
        self.last_face_time = None  # Last time face was seen
        self.tracking_active = False
        
        # Initialize PID controller
        self.pid = PIDController(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            output_min=-(self.max_angle - self.min_angle) / 2,
            output_max=(self.max_angle - self.min_angle) / 2,
            integral_limit=10.0
        )
        
        # Initialize servo driver
        self.servo = ServoDriver(
            gpio_pin=self.servo_gpio_pin,
            min_angle=self.min_angle,
            max_angle=self.max_angle,
            initial_angle=self.neutral_angle,
            simulate=self.simulate_hardware
        )
        self.servo.initialize()
        
        # Subscribe to face bounding box
        self.face_bbox_sub = self.create_subscription(
            Point,
            '/r2d2/perception/face_bbox',
            self.face_bbox_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Subscribe to person status for gating
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Publisher for current servo angle (for monitoring)
        self.angle_pub = self.create_publisher(
            Float32,
            '/r2d2/head_control/tilt_angle',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create timer for periodic processing (handles loss detection)
        timer_period = 1.0 / self.tracking_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f"Tilt Tracking Node initialized:\n"
            f"  Enabled: {self.enabled}\n"
            f"  Servo GPIO: {self.servo_gpio_pin}\n"
            f"  Angle range: {self.min_angle}° - {self.max_angle}° (neutral: {self.neutral_angle}°)\n"
            f"  PID gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}\n"
            f"  Deadband: {self.deadband * 100:.0f}% of frame\n"
            f"  Hardware simulation: {self.servo.simulate}"
        )
    
    def status_callback(self, msg: String):
        """Handle person status updates for tracking gating."""
        try:
            status_data = json.loads(msg.data)
            self.current_status = status_data.get('status', 'blue')
        except json.JSONDecodeError:
            pass
    
    def face_bbox_callback(self, msg: Point):
        """
        Handle face bounding box updates.
        
        Args:
            msg: Point message with x, y (normalized 0-1), z (face size)
        """
        if not self.enabled:
            return
        
        # Only track when status is RED or GREEN (face visible)
        if self.current_status == "blue":
            return
        
        # Update last face time
        self.last_face_time = time.time()
        self.tracking_active = True
        
        # Apply exponential smoothing to face Y position
        face_y = msg.y
        self.smoothed_y = self.smoothing_alpha * face_y + (1.0 - self.smoothing_alpha) * self.smoothed_y
        
        # Calculate error from center (0.5)
        # Positive error = face is below center = need to tilt down (increase angle)
        error_y = self.smoothed_y - 0.5
        
        # Apply deadband - ignore small errors
        if abs(error_y) < self.deadband:
            error_y = 0.0
        
        # Compute PID output (degrees of adjustment)
        adjustment = self.pid.compute(error_y)
        
        # Calculate new angle
        # Positive adjustment = face below center = tilt down = increase angle
        current_angle = self.servo.get_angle()
        new_angle = current_angle + adjustment
        
        # Clamp to valid range
        new_angle = max(self.min_angle, min(self.max_angle, new_angle))
        
        # Command servo
        self.servo.set_angle(new_angle)
        
        # Publish current angle
        angle_msg = Float32()
        angle_msg.data = new_angle
        self.angle_pub.publish(angle_msg)
        
        # Debug logging (throttled)
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 1.0:
                self.get_logger().debug(
                    f"Tracking: face_y={face_y:.3f}, smoothed={self.smoothed_y:.3f}, "
                    f"error={error_y:.3f}, adj={adjustment:.1f}°, angle={new_angle:.1f}°"
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
    
    def timer_callback(self):
        """
        Periodic callback for loss handling and status publishing.
        """
        if not self.enabled:
            return
        
        current_time = time.time()
        
        # Check if face has been lost
        if self.last_face_time is None:
            # Never seen a face - stay at neutral
            return
        
        time_since_face = current_time - self.last_face_time
        
        if time_since_face < self.hold_timeout_sec:
            # Within hold period - keep current position
            return
        
        if time_since_face < self.neutral_timeout_sec:
            # After hold but before neutral timeout - start drifting to neutral
            if self.tracking_active:
                self.get_logger().info("Face lost - holding position briefly")
                self.tracking_active = False
                self.pid.reset()  # Reset PID state
            return
        
        # After neutral timeout - return to neutral position
        current_angle = self.servo.get_angle()
        if abs(current_angle - self.neutral_angle) > 0.5:
            # Slowly move toward neutral
            dt = 1.0 / self.tracking_rate_hz
            max_change = self.neutral_return_speed * dt
            
            if current_angle < self.neutral_angle:
                new_angle = min(current_angle + max_change, self.neutral_angle)
            else:
                new_angle = max(current_angle - max_change, self.neutral_angle)
            
            self.servo.set_angle(new_angle)
            
            # Publish angle
            angle_msg = Float32()
            angle_msg.data = new_angle
            self.angle_pub.publish(angle_msg)
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Tilt Tracking Node shutting down...")
        
        # Return servo to neutral
        self.servo.set_angle(self.neutral_angle)
        time.sleep(0.3)  # Give servo time to move
        
        # Cleanup servo
        self.servo.cleanup()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TiltTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

