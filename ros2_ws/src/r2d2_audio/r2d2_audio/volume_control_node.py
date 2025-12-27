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
  - /r2d2/audio/master_volume (std_msgs/Float32): Master volume 0.0-max_volume_cap

Features:
  - Calibration-based ADC mapping (loads from ~/.r2d2/adc_calibration.json)
  - Maximum volume cap (0.7) based on baseline tests to prevent distortion
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
from typing import Optional, Dict, Any

# Try to import ADC library (may not be available without hardware)
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    ADC_AVAILABLE = True
except ImportError:
    ADC_AVAILABLE = False

# Calibration file paths
CALIBRATION_FILE = Path.home() / '.r2d2' / 'adc_calibration.json'
VOLUME_STATE_FILE = Path.home() / '.r2d2' / 'volume_state.json'

# Default calibration values (16-bit ADC range)
DEFAULT_ADC_MIN = 0
DEFAULT_ADC_MAX = 26400  # ~80% of 32767 (typical for 3.3V reference)

# Maximum volume cap based on baseline tests (distortion starts at 0.7-1.0)
MAX_VOLUME_CAP = 0.7


class VolumeControlNode(Node):
    """
    ROS 2 node for reading physical volume knob and publishing master volume.
    
    Reads from ADS1115 ADC connected to B5K potentiometer and converts
    analog reading to volume percentage (0.0 - max_volume_cap).
    
    Uses calibration data from ~/.r2d2/adc_calibration.json to map ADC
    values to volume. Volume is capped at MAX_VOLUME_CAP (0.7) based on
    baseline tests to prevent audio distortion.
    
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
        self.declare_parameter('dead_zone_high', 0.05)  # 95-100% ADC → max volume
        self.declare_parameter('master_volume_default', 0.35)  # Default if no ADC (middle of range)
        self.declare_parameter('max_volume_cap', MAX_VOLUME_CAP)  # Max volume to prevent distortion
        self.declare_parameter('persistence_file', str(VOLUME_STATE_FILE))
        self.declare_parameter('calibration_file', str(CALIBRATION_FILE))
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
        self.max_volume_cap = self.get_parameter('max_volume_cap').value
        self.persistence_file = Path(self.get_parameter('persistence_file').value)
        self.calibration_file = Path(self.get_parameter('calibration_file').value)
        self.hardware_enabled = self.get_parameter('hardware_enabled').value
        
        # Calibration values (loaded from file or defaults)
        self.adc_min = DEFAULT_ADC_MIN
        self.adc_max = DEFAULT_ADC_MAX
        self.calibration_loaded = False
        
        # State variables
        self.current_volume = self.default_volume
        self.smoothed_adc = None
        self.last_published_volume = None
        self.adc: Optional[ADS.ADS1115] = None
        self.adc_channel_obj: Optional[AnalogIn] = None
        self.hardware_working = False
        
        # Load calibration data
        self._load_calibration()
        
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
            f"  Calibration: {'LOADED' if self.calibration_loaded else 'DEFAULTS'}\n"
            f"  ADC Range: {self.adc_min} - {self.adc_max}\n"
            f"  Max Volume Cap: {self.max_volume_cap:.2f}\n"
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
                # Read raw ADC value (0-32767 for signed 16-bit from ADS1115)
                raw_value = self.adc_channel_obj.value
                
                # Map using calibration values
                normalized = self._map_adc_to_normalized(raw_value)
                
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
                
                # Apply dead zones and max volume cap
                volume = self._apply_dead_zones(normalized)
                
                # Update current volume
                self.current_volume = volume
                
            except Exception as e:
                self.get_logger().error(f"ADC read error: {e}")
                # Keep using last known volume
        
        # Publish if changed significantly
        self._publish_volume(self.current_volume)
    
    def _map_adc_to_normalized(self, raw_value: int) -> float:
        """
        Map raw ADC value to normalized 0.0-1.0 using calibration.
        
        Uses calibration values (adc_min, adc_max) to map the raw ADC
        reading to a normalized value.
        
        Args:
            raw_value: Raw ADC value from ADS1115
        
        Returns:
            Normalized value (0.0-1.0)
        """
        # Clamp to calibrated range
        clamped = max(self.adc_min, min(self.adc_max, raw_value))
        
        # Avoid division by zero
        adc_range = self.adc_max - self.adc_min
        if adc_range <= 0:
            return 0.0
        
        # Linear mapping
        normalized = (clamped - self.adc_min) / adc_range
        return max(0.0, min(1.0, normalized))
    
    def _apply_dead_zones(self, normalized: float) -> float:
        """
        Apply dead zones to normalized ADC value and cap at max volume.
        
        Maps:
        - 0.0 to dead_zone_low → 0.0 (mute)
        - dead_zone_low to (1.0 - dead_zone_high) → 0.0 to max_volume_cap (linear)
        - (1.0 - dead_zone_high) to 1.0 → max_volume_cap
        
        Args:
            normalized: Normalized ADC value (0.0-1.0)
        
        Returns:
            Volume value with dead zones applied (0.0 - max_volume_cap)
        """
        if normalized <= self.dead_zone_low:
            return 0.0
        elif normalized >= (1.0 - self.dead_zone_high):
            return self.max_volume_cap
        else:
            # Linear mapping between dead zones to 0.0 - max_volume_cap
            active_range = 1.0 - self.dead_zone_low - self.dead_zone_high
            fraction = (normalized - self.dead_zone_low) / active_range
            return fraction * self.max_volume_cap
    
    def _publish_volume(self, volume: float, force: bool = False):
        """
        Publish volume if it has changed significantly.
        
        Args:
            volume: Volume to publish (0.0 - max_volume_cap)
            force: Publish even if unchanged
        """
        # Check if change is significant
        if not force and self.last_published_volume is not None:
            change = abs(volume - self.last_published_volume)
            if change < self.min_change:
                return
        
        # Clamp to valid range (0.0 to max_volume_cap)
        volume = max(0.0, min(self.max_volume_cap, volume))
        
        # Publish
        msg = Float32()
        msg.data = volume
        self.volume_pub.publish(msg)
        
        self.last_published_volume = volume
        
        # Save to persistence file
        self._save_volume_state(volume)
        
        self.get_logger().debug(f"Published volume: {volume:.3f}")
    
    def _load_calibration(self):
        """
        Load ADC calibration data from file.
        
        Calibration file is created by test_adc_calibration.py --calibrate
        and contains the min/max ADC values for the potentiometer.
        """
        if self.calibration_file.exists():
            try:
                with open(self.calibration_file, 'r') as f:
                    calibration = json.load(f)
                
                if calibration.get('calibrated', False):
                    self.adc_min = calibration.get('adc_min', DEFAULT_ADC_MIN)
                    self.adc_max = calibration.get('adc_max', DEFAULT_ADC_MAX)
                    
                    # Override max_volume if specified in calibration
                    if 'max_volume' in calibration:
                        self.max_volume_cap = calibration['max_volume']
                    
                    self.calibration_loaded = True
                    self.get_logger().info(
                        f"Loaded ADC calibration from {self.calibration_file}\n"
                        f"  ADC Range: {self.adc_min} - {self.adc_max}\n"
                        f"  Max Volume: {self.max_volume_cap}"
                    )
                else:
                    self.get_logger().warn(
                        f"Calibration file exists but not calibrated. "
                        f"Run: python3 test_adc_calibration.py --calibrate"
                    )
            except Exception as e:
                self.get_logger().warn(f"Could not load calibration: {e}")
        else:
            self.get_logger().info(
                f"No calibration file found at {self.calibration_file}\n"
                f"Using defaults: ADC {self.adc_min}-{self.adc_max}, max vol {self.max_volume_cap}\n"
                f"Run: python3 test_adc_calibration.py --calibrate"
            )
    
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

