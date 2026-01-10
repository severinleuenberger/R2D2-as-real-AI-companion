#!/usr/bin/env python3
"""
GPIO Status LED Node - Direct GPIO control via NPN transistors

Controls 3 status LEDs via GPIO pins using NPN transistors (2N2222):
  - RED LED (Pin 7): Person recognized status
  - BLUE LED (Pin 11): No person / idle status
  - YELLOW LED (Pin 13): Gesture flash indicator

This replaces the MCP23017 I2C approach which failed due to Jetson AGX Orin
40-pin header I2C pinmux issues.

Hardware:
  - NPN transistors (2N2222) switch 5V power to LEDs
  - GPIO provides 3.3V signal to transistor base
  - LEDs connected to 5V via transistor collectors
  - 1kΩ base resistors, 220Ω LED resistors

Author: Severin Leuenberger
Date: January 10, 2026
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading

# Try to import Jetson.GPIO
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: Jetson.GPIO not available, running in simulation mode")


class GPIOStatusLEDNode(Node):
    """ROS2 node to control status LEDs via GPIO pins"""
    
    # GPIO Pin assignments (BOARD numbering)
    PIN_RED = 7      # Red LED - person recognized
    PIN_BLUE = 11    # Blue LED - no person / idle
    PIN_YELLOW = 13  # Yellow LED - gesture flash
    
    def __init__(self):
        super().__init__('gpio_status_led_node')
        
        # Declare parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('simulate', False)
        self.declare_parameter('gesture_flash_duration_ms', 500)
        
        # Get parameters
        self.enabled = self.get_parameter('enabled').value
        force_simulate = self.get_parameter('simulate').value
        self.gesture_flash_duration = self.get_parameter('gesture_flash_duration_ms').value / 1000.0
        
        # Determine if we can use real GPIO
        self.simulation_mode = force_simulate or not GPIO_AVAILABLE
        
        if self.simulation_mode:
            self.get_logger().info('Running in SIMULATION mode (no hardware control)')
        else:
            self.get_logger().info('Running with REAL GPIO hardware')
            self._setup_gpio()
        
        # Subscribe to topics
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self._person_status_callback,
            10
        )
        
        self.gesture_sub = self.create_subscription(
            String,
            '/r2d2/perception/gesture_event',
            self._gesture_event_callback,
            10
        )
        
        # Gesture flash timer
        self.gesture_timer = None
        self.gesture_lock = threading.Lock()
        
        # Current status tracking
        self.current_status = 'blue'  # Start with idle/blue
        
        # Set initial state
        self._set_status_leds(self.current_status)
        
        self.get_logger().info('GPIO Status LED Node started')
        self.get_logger().info(f'  RED LED: Pin {self.PIN_RED}')
        self.get_logger().info(f'  BLUE LED: Pin {self.PIN_BLUE}')
        self.get_logger().info(f'  YELLOW LED: Pin {self.PIN_YELLOW}')
        self.get_logger().info(f'  Gesture flash duration: {self.gesture_flash_duration}s')
    
    def _setup_gpio(self):
        """Initialize GPIO pins"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            # Setup output pins (initially LOW/off)
            GPIO.setup(self.PIN_RED, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.PIN_BLUE, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.PIN_YELLOW, GPIO.OUT, initial=GPIO.LOW)
            
            self.get_logger().info('GPIO pins initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self.simulation_mode = True
    
    def _person_status_callback(self, msg):
        """Handle person status updates"""
        try:
            data = json.loads(msg.data)
            status = data.get('status', '').lower()
            person_identity = data.get('person_identity', 'unknown')
            
            if status in ['red', 'blue', 'green']:
                self.get_logger().debug(f'Status update: {status} ({person_identity})')
                self.current_status = status
                self._set_status_leds(status)
            else:
                self.get_logger().warning(f'Unknown status: {status}')
        
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in person_status: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing status: {e}')
    
    def _gesture_event_callback(self, msg):
        """Handle gesture flash events"""
        gesture = msg.data
        self.get_logger().info(f'Gesture detected: {gesture} -> Yellow flash')
        
        # Flash yellow LED
        self._flash_yellow()
    
    def _set_status_leds(self, status):
        """Set status LEDs (RED/BLUE) - mutually exclusive"""
        if not self.enabled:
            return
        
        if status == 'red':
            self._set_led(self.PIN_RED, True)
            self._set_led(self.PIN_BLUE, False)
            self.get_logger().debug('🔴 RED LED ON (person recognized)')
        elif status == 'blue':
            self._set_led(self.PIN_RED, False)
            self._set_led(self.PIN_BLUE, True)
            self.get_logger().debug('🔵 BLUE LED ON (no person)')
        elif status == 'green':
            # We only have 2 status LEDs, treat green as blue for now
            self.get_logger().warning('GREEN status -> using BLUE LED (no green LED available)')
            self._set_led(self.PIN_RED, False)
            self._set_led(self.PIN_BLUE, True)
        else:
            # Unknown status - turn off both
            self._set_led(self.PIN_RED, False)
            self._set_led(self.PIN_BLUE, False)
    
    def _flash_yellow(self):
        """Flash yellow LED for gesture indication"""
        if not self.enabled:
            return
        
        with self.gesture_lock:
            # Cancel previous timer if any
            if self.gesture_timer is not None:
                self.gesture_timer.cancel()
            
            # Turn on yellow LED
            self._set_led(self.PIN_YELLOW, True)
            
            # Schedule turn off
            self.gesture_timer = threading.Timer(
                self.gesture_flash_duration,
                lambda: self._set_led(self.PIN_YELLOW, False)
            )
            self.gesture_timer.start()
    
    def _set_led(self, pin, state):
        """Set LED state (True=ON, False=OFF)"""
        if self.simulation_mode:
            led_name = {
                self.PIN_RED: 'RED',
                self.PIN_BLUE: 'BLUE',
                self.PIN_YELLOW: 'YELLOW'
            }.get(pin, 'UNKNOWN')
            state_str = 'ON' if state else 'OFF'
            self.get_logger().debug(f'[SIM] {led_name} LED -> {state_str}')
        else:
            try:
                GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                self.get_logger().error(f'Failed to set GPIO pin {pin}: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down GPIO Status LED Node')
        
        # Cancel gesture timer if running
        if self.gesture_timer is not None:
            self.gesture_timer.cancel()
        
        # Turn off all LEDs
        if not self.simulation_mode:
            try:
                self._set_led(self.PIN_RED, False)
                self._set_led(self.PIN_BLUE, False)
                self._set_led(self.PIN_YELLOW, False)
                GPIO.cleanup()
                self.get_logger().info('GPIO cleaned up')
            except Exception as e:
                self.get_logger().error(f'Error during GPIO cleanup: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPIOStatusLEDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
