#!/usr/bin/env python3
"""
R2D2 Volume Control Node - Physical Potentiometer-based Master Volume

Reads analog values from a B5K potentiometer via ADS1115 ADC and publishes
master volume to ROS2 topic for all audio nodes to consume.

Hardware:
  - B5K (5kΩ) Linear Potentiometer
  - ADS1115 16-bit ADC (I2C)
  - Jetson AGX Orin I2C bus

Publishes:
  - /r2d2/audio/master_volume (std_msgs/Float32): Master volume 0.0-1.0

Features:
  - Exponential smoothing to prevent jitter
  - Dead zones at min/max positions
  - Volume persistence across restarts
  - Fallback mode without hardware (uses default volume)

Author: R2D2 Audio System
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import json
import time
from pathlib import Path
from typing import Optional

# Try to import ADC library (may not be available without hardware)
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    ADC_AVAILABLE = True
except ImportError:
    ADC_AVAILABLE = False


class VolumeControlNode(Node):
    """
    ROS 2 node for reading physical volume knob and publishing master volume.
    
    Reads from ADS1115 ADC connected to B5K potentiometer and converts
    analog reading to volume percentage (0.0-1.0).
    
    Publishes to /r2d2/audio/master_volume for all audio nodes to subscribe.
    """
    
    def __init__(self):
        super().__init__('volume_control_node')
        
        self.get_logger().info("R2D2 Volume Control Node starting...")
        
        # Declare parameters
        self.declare_parameter('adc_i2c_bus', 1)  # I2C bus number (0-8 on Jetson)
        self.declare_parameter('adc_address', 0x48)  # ADS1115 default address
        self.declare_parameter('adc_channel', 0)  # ADC channel (0-3)
        self.declare_parameter('poll_rate_hz', 10.0)  # How often to read ADC
        self.declare_parameter('volume_smoothing', True)  # Enable smoothing
        self.declare_parameter('smoothing_alpha', 0.2)  # EMA alpha (0.0-1.0)
        self.declare_parameter('min_change_threshold', 0.01)  # 1% minimum change
        self.declare_parameter('dead_zone_low', 0.05)  # 0-5% ADC → 0.0 volume
        self.declare_parameter('dead_zone_high', 0.05)  # 95-100% ADC → 1.0 volume
        self.declare_parameter('master_volume_default', 0.5)  # Default if no ADC
        self.declare_parameter('persistence_file', str(Path.home() / '.r2d2' / 'volume_state.json'))
        self.declare_parameter('hardware_enabled', True)  # Set False for testing without hardware
        
        # Get parameters
        self.i2c_bus = self.get_parameter('adc_i2c_bus').value
        self.adc_address = self.get_parameter('adc_address').value
        self.adc_channel = self.get_parameter('adc_channel').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.smoothing_enabled = self.get_parameter('volume_smoothing').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.min_change = self.get_parameter('min_change_threshold').value
        self.dead_zone_low = self.get_parameter('dead_zone_low').value
        self.dead_zone_high = self.get_parameter('dead_zone_high').value
        self.default_volume = self.get_parameter('master_volume_default').value
        self.persistence_file = Path(self.get_parameter('persistence_file').value)
        self.hardware_enabled = self.get_parameter('hardware_enabled').value
        
        # State variables
        self.current_volume = self.default_volume
        self.smoothed_adc = None
        self.last_published_volume = None
        self.adc: Optional[ADS.ADS1115] = None
        self.adc_channel_obj: Optional[AnalogIn] = None
        self.hardware_working = False
        
        # Load persisted volume
        self._load_volume_state()
        
        # Initialize ADC
        if self.hardware_enabled:
            self._init_adc()
        else:
            self.get_logger().warn("Hardware disabled - using software-only mode")
        
        # Create publisher
        self.volume_pub = self.create_publisher(
            Float32,
            '/r2d2/audio/master_volume',
            qos_profile=rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL  # Late subscribers get last value
            )
        )
        
        # Create timer for ADC polling
        timer_period = 1.0 / self.poll_rate
        self.poll_timer = self.create_timer(timer_period, self._poll_callback)
        
        # Publish initial volume immediately
        self._publish_volume(self.current_volume, force=True)
        
        self.get_logger().info(
            f"Volume Control Node initialized:\n"
            f"  Hardware: {'ENABLED' if self.hardware_enabled else 'DISABLED'}\n"
            f"  ADC Working: {self.hardware_working}\n"
            f"  I2C Bus: {self.i2c_bus}\n"
            f"  ADC Address: 0x{self.adc_address:02X}\n"
            f"  Poll Rate: {self.poll_rate} Hz\n"
            f"  Smoothing: {self.smoothing_enabled} (alpha={self.smoothing_alpha})\n"
            f"  Dead Zones: low={self.dead_zone_low}, high={self.dead_zone_high}\n"
            f"  Initial Volume: {self.current_volume:.2f}\n"
            f"  Persistence: {self.persistence_file}"
        )
    
    def _init_adc(self):
        """Initialize ADS1115 ADC hardware."""
        if not ADC_AVAILABLE:
            self.get_logger().warn(
                "ADC library not available. Install with: "
                "pip3 install adafruit-circuitpython-ads1x15"
            )
            return
        
        try:
            # Create I2C bus
            # Note: On Jetson, we might need to specify the bus differently
            # For now, try using board.I2C() which usually works
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Create ADC object
            self.adc = ADS.ADS1115(i2c, address=self.adc_address)
            
            # Create channel object
            if self.adc_channel == 0:
                self.adc_channel_obj = AnalogIn(self.adc, ADS.P0)
            elif self.adc_channel == 1:
                self.adc_channel_obj = AnalogIn(self.adc, ADS.P1)
            elif self.adc_channel == 2:
                self.adc_channel_obj = AnalogIn(self.adc, ADS.P2)
            elif self.adc_channel == 3:
                self.adc_channel_obj = AnalogIn(self.adc, ADS.P3)
            else:
                raise ValueError(f"Invalid ADC channel: {self.adc_channel}")
            
            # Test read
            test_value = self.adc_channel_obj.value
            self.get_logger().info(f"ADC initialized successfully. Test read: {test_value}")
            self.hardware_working = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ADC: {e}")
            self.get_logger().warn("Continuing in software-only mode (using default volume)")
            self.hardware_working = False
    
    def _poll_callback(self):
        """Timer callback: read ADC and publish volume if changed."""
        if self.hardware_working and self.adc_channel_obj is not None:
            try:
                # Read raw ADC value (0-65535 for 16-bit)
                raw_value = self.adc_channel_obj.value
                
                # Normalize to 0.0-1.0
                normalized = raw_value / 65535.0
                
                # Apply smoothing
                if self.smoothing_enabled:
                    if self.smoothed_adc is None:
                        self.smoothed_adc = normalized
                    else:
                        self.smoothed_adc = (
                            self.smoothing_alpha * normalized +
                            (1 - self.smoothing_alpha) * self.smoothed_adc
                        )
                    normalized = self.smoothed_adc
                
                # Apply dead zones
                volume = self._apply_dead_zones(normalized)
                
                # Update current volume
                self.current_volume = volume
                
            except Exception as e:
                self.get_logger().error(f"ADC read error: {e}")
                # Keep using last known volume
        
        # Publish if changed significantly
        self._publish_volume(self.current_volume)
    
    def _apply_dead_zones(self, normalized: float) -> float:
        """
        Apply dead zones to normalized ADC value.
        
        Maps:
        - 0.0 to dead_zone_low → 0.0
        - dead_zone_low to (1.0 - dead_zone_high) → 0.0 to 1.0 (linear)
        - (1.0 - dead_zone_high) to 1.0 → 1.0
        
        Args:
            normalized: Normalized ADC value (0.0-1.0)
        
        Returns:
            Volume value with dead zones applied (0.0-1.0)
        """
        if normalized <= self.dead_zone_low:
            return 0.0
        elif normalized >= (1.0 - self.dead_zone_high):
            return 1.0
        else:
            # Linear mapping between dead zones
            active_range = 1.0 - self.dead_zone_low - self.dead_zone_high
            return (normalized - self.dead_zone_low) / active_range
    
    def _publish_volume(self, volume: float, force: bool = False):
        """
        Publish volume if it has changed significantly.
        
        Args:
            volume: Volume to publish (0.0-1.0)
            force: Publish even if unchanged
        """
        # Check if change is significant
        if not force and self.last_published_volume is not None:
            change = abs(volume - self.last_published_volume)
            if change < self.min_change:
                return
        
        # Clamp to valid range
        volume = max(0.0, min(1.0, volume))
        
        # Publish
        msg = Float32()
        msg.data = volume
        self.volume_pub.publish(msg)
        
        self.last_published_volume = volume
        
        # Save to persistence file
        self._save_volume_state(volume)
        
        self.get_logger().debug(f"Published volume: {volume:.3f}")
    
    def _load_volume_state(self):
        """Load volume state from persistence file."""
        try:
            if self.persistence_file.exists():
                with open(self.persistence_file, 'r') as f:
                    state = json.load(f)
                    self.current_volume = state.get('volume', self.default_volume)
                    self.get_logger().info(f"Loaded persisted volume: {self.current_volume:.2f}")
        except Exception as e:
            self.get_logger().warn(f"Could not load volume state: {e}")
    
    def _save_volume_state(self, volume: float):
        """Save volume state to persistence file."""
        try:
            self.persistence_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self.persistence_file, 'w') as f:
                json.dump({
                    'volume': volume,
                    'timestamp': time.time()
                }, f)
        except Exception as e:
            self.get_logger().warn(f"Could not save volume state: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = VolumeControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Volume Control Node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

