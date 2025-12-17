#!/usr/bin/env python3
"""
Status LED Controller - Visual feedback for person recognition state.

Subscribes to /r2d2/audio/person_status (JSON String message) and controls LED colors:
  - RED (GPIO): Target person recognized (active engagement)
  - BLUE (GPIO): No person recognized (idle/waiting)
  - GREEN (GPIO): Unknown person detected (caution)

This provides real-time visual feedback synchronized with audio alerts
and feeds into the dialogue system for context awareness.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class StatusLEDNode(Node):
    """
    Controls RGB LED based on face recognition status.
    
    Supports both physical GPIO (Jetson) and simulation mode.
    """
    
    def __init__(self):
        super().__init__('status_led_controller')
        
        self.get_logger().info("Status LED Controller initializing...")
        
        # Declare parameters
        self.declare_parameter('led_mode', 'white')    # 'white' or 'rgb' mode
        self.declare_parameter('led_pin_white', 17)    # GPIO pin for white LED (Pin 22 on 40-pin header)
        self.declare_parameter('led_pin_red', 17)      # GPIO pin for red LED (backward compatibility)
        self.declare_parameter('led_pin_green', 27)    # GPIO pin for green LED (backward compatibility)
        self.declare_parameter('led_pin_blue', 22)     # GPIO pin for blue LED (backward compatibility)
        self.declare_parameter('brightness', 1.0)      # 0.0-1.0
        self.declare_parameter('update_rate_hz', 10)
        self.declare_parameter('enabled', True)
        self.declare_parameter('simulate_gpio', not HAS_GPIO)  # Auto-detect simulation mode
        
        # Get parameters
        self.led_mode = self.get_parameter('led_mode').value
        self.led_pin_white = self.get_parameter('led_pin_white').value
        self.led_pins = {
            'red': self.get_parameter('led_pin_red').value,
            'green': self.get_parameter('led_pin_green').value,
            'blue': self.get_parameter('led_pin_blue').value,
        }
        self.brightness = self.get_parameter('brightness').value
        self.simulate_gpio = self.get_parameter('simulate_gpio').value
        
        # Current status
        self.current_status = "blue"
        self.current_person = "no_person"
        self.last_status_update = time.time()
        
        # Setup GPIO
        if not self.simulate_gpio and HAS_GPIO:
            self._setup_gpio()
        else:
            if self.simulate_gpio:
                self.get_logger().warn("🔶 Running in GPIO simulation mode (no physical hardware)")
            else:
                self.get_logger().warn("🔶 RPi.GPIO not available - running in simulation mode")
        
        # Subscribe to status (JSON String messages)
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        if self.led_mode == 'white':
            self.get_logger().info(
                f"Status LED Controller initialized:\n"
                f"  LED Mode: WHITE (single LED on/off control)\n"
                f"  Hardware Mode: {'GPIO Hardware' if (not self.simulate_gpio and HAS_GPIO) else 'Simulation'}\n"
                f"  White LED GPIO: {self.led_pin_white} (Physical Pin 22 on 40-pin header)\n"
                f"  State Mapping: RED=ON, BLUE/GREEN=OFF\n"
                f"  Brightness: {self.brightness*100:.0f}%\n"
                f"  Enabled: {self.get_parameter('enabled').value}"
            )
        else:
            self.get_logger().info(
                f"Status LED Controller initialized:\n"
                f"  LED Mode: RGB (separate color control)\n"
                f"  Hardware Mode: {'GPIO Hardware' if (not self.simulate_gpio and HAS_GPIO) else 'Simulation'}\n"
                f"  Red LED GPIO: {self.led_pins['red']}\n"
                f"  Green LED GPIO: {self.led_pins['green']}\n"
                f"  Blue LED GPIO: {self.led_pins['blue']}\n"
                f"  Brightness: {self.brightness*100:.0f}%\n"
                f"  Enabled: {self.get_parameter('enabled').value}"
            )
    
    def _setup_gpio(self):
        """Initialize GPIO pins for LED control."""
        if not HAS_GPIO:
            return
        
        try:
            GPIO.setmode(GPIO.BCM)
            
            if self.led_mode == 'white':
                # White LED mode: only initialize the single white LED pin
                GPIO.setup(self.led_pin_white, GPIO.OUT)
                GPIO.output(self.led_pin_white, GPIO.LOW)  # Start off
                self.get_logger().info(f"✅ GPIO pin {self.led_pin_white} initialized for white LED control")
            else:
                # RGB mode: initialize all three color pins
                for color, pin in self.led_pins.items():
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.LOW)  # Start off
                self.get_logger().info("✅ GPIO pins initialized for RGB LED control")
        except Exception as e:
            self.get_logger().error(f"Failed to setup GPIO: {e}")
            self.simulate_gpio = True
    
    def status_callback(self, msg: String):
        """
        Update LED when status changes.
        
        Args:
            msg: String message containing JSON PersonStatus data
        """
        try:
            status_data = json.loads(msg.data)
            self.current_status = status_data.get('status', 'blue')
            self.current_person = status_data.get('person_identity', 'no_person')
            self.last_status_update = time.time()
            
            self._update_led()
            
            duration = status_data.get('duration_in_state', 0)
            confidence = status_data.get('confidence', 0)
            self.get_logger().debug(
                f"🔄 LED Update: {self.current_status.upper()} | Person: {self.current_person} | "
                f"Confidence: {confidence*100:.0f}% | Duration: {duration:.1f}s"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")
    
    def _update_led(self):
        """Set LED color based on current status."""
        if not self.get_parameter('enabled').value:
            self._all_off()
            return
        
        if self.led_mode == 'white':
            # White LED mode: ON for RED state, OFF for BLUE/GREEN states
            if self.current_status == "red":
                # RED: Target person recognized (ACTIVE ENGAGEMENT) - LED ON
                self._set_white_led(True)
                if not self.simulate_gpio:
                    self.get_logger().debug("💡 White LED: ON (Person recognized - RED state)")
            else:
                # BLUE/GREEN: No person or unknown person - LED OFF
                self._set_white_led(False)
                if not self.simulate_gpio:
                    status_name = "BLUE (Idle)" if self.current_status == "blue" else "GREEN (Unknown)"
                    self.get_logger().debug(f"⚫ White LED: OFF ({status_name} state)")
        else:
            # RGB mode: original behavior
            if self.current_status == "red":
                # RED: Target person recognized (ACTIVE ENGAGEMENT)
                self._set_color('red')
                if not self.simulate_gpio:
                    self.get_logger().debug("🔴 LED: RED (Person recognized)")
            
            elif self.current_status == "blue":
                # BLUE: No person recognized (IDLE/WAITING)
                self._set_color('blue')
                if not self.simulate_gpio:
                    self.get_logger().debug("🔵 LED: BLUE (Idle, awaiting)")
            
            elif self.current_status == "green":
                # GREEN: Unknown person detected (CAUTION)
                self._set_color('green')
                if not self.simulate_gpio:
                    self.get_logger().debug("🟢 LED: GREEN (Unknown person)")
    
    def _set_white_led(self, state: bool):
        """
        Set white LED on or off.
        
        Args:
            state: True for ON (GPIO HIGH), False for OFF (GPIO LOW)
        """
        if self.simulate_gpio or not HAS_GPIO:
            # Simulation mode - just log
            return
        
        try:
            GPIO.output(self.led_pin_white, GPIO.HIGH if state else GPIO.LOW)
        except Exception as e:
            self.get_logger().error(f"Error setting white LED: {e}")
    
    def _set_color(self, color: str):
        """
        Set LED to specified color (RGB mode only).
        
        Args:
            color: "red", "green", or "blue"
        """
        if self.simulate_gpio or not HAS_GPIO:
            # Simulation mode - just log
            return
        
        try:
            # Turn off all LEDs first
            for pin in self.led_pins.values():
                GPIO.output(pin, GPIO.LOW)
            
            # Turn on specified color
            if color in self.led_pins:
                GPIO.output(self.led_pins[color], GPIO.HIGH)
        except Exception as e:
            self.get_logger().error(f"Error setting LED color: {e}")
    
    def _all_off(self):
        """Turn all LEDs off."""
        if self.simulate_gpio or not HAS_GPIO:
            return
        
        try:
            if self.led_mode == 'white':
                # White LED mode: turn off single white LED
                GPIO.output(self.led_pin_white, GPIO.LOW)
            else:
                # RGB mode: turn off all color LEDs
                for pin in self.led_pins.values():
                    GPIO.output(pin, GPIO.LOW)
        except Exception as e:
            self.get_logger().error(f"Error turning off LEDs: {e}")
    
    def __del__(self):
        """Cleanup GPIO on shutdown."""
        if not self.simulate_gpio and HAS_GPIO:
            try:
                self._all_off()
                GPIO.cleanup()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = StatusLEDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Status LED Controller shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
