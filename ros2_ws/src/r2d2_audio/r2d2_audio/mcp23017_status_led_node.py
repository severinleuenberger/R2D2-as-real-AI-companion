#!/usr/bin/env python3
"""
MCP23017 Status LED Controller - Multi-color visual feedback for R2D2 status.

Controls 4 individual LEDs via MCP23017 I2C GPIO expander:
  - RED LED: Person recognized (status="red")
  - BLUE LED: No person (status="blue")
  - GREEN LED: Unknown person (status="green")
  - YELLOW LED: Gesture detected (500ms flash)

Subscribes to:
  - /r2d2/audio/person_status (String JSON) - Person recognition state
  - /r2d2/perception/gesture_event (String) - Hand gesture events

Hardware:
  - MCP23017 I2C GPIO Expander at address 0x20
  - Jetson I2C bus (Pins 3, 5, 1, 6)
  - 4 LEDs connected to PA0-PA3 outputs

Date: January 9, 2026
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading

try:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017
    HAS_MCP23017 = True
except ImportError:
    HAS_MCP23017 = False


class MCP23017StatusLEDNode(Node):
    """
    ROS2 node for controlling status LEDs via MCP23017 I2C GPIO expander.
    
    Provides visual feedback for person recognition status and gesture detection.
    """
    
    def __init__(self):
        super().__init__('mcp23017_status_led_node')
        
        self.get_logger().info("MCP23017 Status LED Node initializing...")
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x20)
        self.declare_parameter('enabled', True)
        self.declare_parameter('simulate', not HAS_MCP23017)
        self.declare_parameter('gesture_flash_duration_ms', 500)
        
        # Pin mapping parameters (allows configuration for reversed wiring)
        self.declare_parameter('pa0_controls', 'red')      # Which color PA0 controls
        self.declare_parameter('pa1_controls', 'blue')     # Which color PA1 controls
        self.declare_parameter('pa2_controls', 'green')    # Which color PA2 controls
        self.declare_parameter('pa3_controls', 'yellow')   # Which color PA3 controls
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.enabled = self.get_parameter('enabled').value
        self.simulate = self.get_parameter('simulate').value
        self.gesture_flash_duration = self.get_parameter('gesture_flash_duration_ms').value / 1000.0
        
        # Get pin mapping
        self.pin_mapping = {
            'pa0': self.get_parameter('pa0_controls').value,
            'pa1': self.get_parameter('pa1_controls').value,
            'pa2': self.get_parameter('pa2_controls').value,
            'pa3': self.get_parameter('pa3_controls').value,
        }
        
        # Current status
        self.current_status = "blue"
        self.current_person = "no_person"
        self.last_status_update = time.time()
        
        # LED pins and MCP23017
        self.mcp = None
        self.led_pins = {}  # Maps color → pin object
        self.gesture_flash_timer = None
        
        # Initialize hardware
        if not self.simulate and HAS_MCP23017:
            self._init_mcp23017()
        else:
            if self.simulate:
                self.get_logger().warn("🔶 Running in simulation mode (no hardware)")
            else:
                self.get_logger().warn("🔶 MCP23017 library not available - simulation mode")
        
        # Subscribe to person status
        self.status_sub = self.create_subscription(
            String,
            '/r2d2/audio/person_status',
            self.status_callback,
            10
        )
        
        # Subscribe to gesture events
        self.gesture_sub = self.create_subscription(
            String,
            '/r2d2/perception/gesture_event',
            self.gesture_callback,
            10
        )
        
        # Log configuration
        pin_config = ", ".join([f"PA{i}={color}" for i, (pa, color) in enumerate(self.pin_mapping.items())])
        self.get_logger().info(
            f"✅ MCP23017 Status LED Node ready\n"
            f"   I2C Address: 0x{self.i2c_address:02x}\n"
            f"   Pin Mapping: {pin_config}\n"
            f"   Mode: {'Simulation' if self.simulate else 'Hardware'}\n"
            f"   Subscribed to: /r2d2/audio/person_status, /r2d2/perception/gesture_event"
        )
    
    def _init_mcp23017(self):
        """Initialize MCP23017 hardware and configure LED pins."""
        try:
            # Initialize I2C bus
            self.get_logger().info(f"Initializing I2C bus {self.i2c_bus}...")
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize MCP23017
            self.get_logger().info(f"Connecting to MCP23017 at 0x{self.i2c_address:02x}...")
            self.mcp = MCP23017(i2c, address=self.i2c_address)
            
            # Configure PA0-PA3 as outputs based on mapping
            self.get_logger().info("Configuring LED pins...")
            
            # Create pin objects
            pa_pins = {
                'pa0': self.mcp.get_pin(0),
                'pa1': self.mcp.get_pin(1),
                'pa2': self.mcp.get_pin(2),
                'pa3': self.mcp.get_pin(3),
            }
            
            # Map to colors based on configuration
            for pa_name, color in self.pin_mapping.items():
                pin = pa_pins[pa_name]
                pin.direction = digitalio.Direction.OUTPUT
                pin.value = False  # Start with all LEDs off
                self.led_pins[color] = pin
                self.get_logger().info(f"  {pa_name.upper()} → {color.upper()} LED")
            
            # Verify we have all required colors
            required_colors = ['red', 'blue', 'green', 'yellow']
            for color in required_colors:
                if color not in self.led_pins:
                    self.get_logger().error(f"❌ Missing LED color: {color}")
                    self.get_logger().error("   Check pin mapping configuration!")
                    self.simulate = True
                    return
            
            self.get_logger().info("✅ MCP23017 initialized successfully - 4 LEDs configured")
            
        except ValueError as e:
            self.get_logger().error(f"❌ Could not find MCP23017 at 0x{self.i2c_address:02x}: {str(e)}")
            self.get_logger().error("   Falling back to simulation mode")
            self.simulate = True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error initializing MCP23017: {str(e)}")
            self.get_logger().error("   Falling back to simulation mode")
            self.simulate = True
    
    def status_callback(self, msg: String):
        """
        Handle person recognition status updates.
        
        Updates status LEDs: RED, BLUE, or GREEN (mutually exclusive).
        
        Args:
            msg: String message containing JSON PersonStatus data
        """
        if not self.enabled:
            return
        
        try:
            status_data = json.loads(msg.data)
            new_status = status_data.get('status', 'blue')
            self.current_person = status_data.get('person_identity', 'no_person')
            
            # Only update if status changed
            if new_status != self.current_status:
                self.current_status = new_status
                self.last_status_update = time.time()
                self._update_status_leds()
                
                # Log status change
                duration = status_data.get('duration_in_state', 0)
                confidence = status_data.get('confidence', 0)
                self.get_logger().info(
                    f"🔄 Status: {self.current_status.upper()} | "
                    f"Person: {self.current_person} | "
                    f"Confidence: {confidence*100:.0f}% | "
                    f"Duration: {duration:.1f}s"
                )
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in status_callback: {e}")
    
    def gesture_callback(self, msg: String):
        """
        Handle gesture detection events.
        
        Flashes yellow LED for 500ms when any gesture is detected.
        
        Args:
            msg: String message containing gesture name
        """
        if not self.enabled:
            return
        
        gesture_name = msg.data
        self.get_logger().info(f"👋 Gesture detected: {gesture_name} → Yellow LED flash")
        
        self._flash_yellow_led()
    
    def _update_status_leds(self):
        """
        Update status LEDs based on current status.
        
        Only ONE status LED is on at a time (mutually exclusive):
        - RED status → Red LED ON, Blue/Green OFF
        - BLUE status → Blue LED ON, Red/Green OFF
        - GREEN status → Green LED ON, Red/Blue OFF
        """
        if not self.enabled:
            self._all_status_leds_off()
            return
        
        if not self.simulate and self.led_pins:
            try:
                # Turn off all status LEDs first
                self.led_pins['red'].value = False
                self.led_pins['blue'].value = False
                self.led_pins['green'].value = False
                
                # Turn on the appropriate status LED
                if self.current_status == "red":
                    self.led_pins['red'].value = True
                    self.get_logger().debug("🔴 RED LED ON (Person recognized)")
                    
                elif self.current_status == "blue":
                    self.led_pins['blue'].value = True
                    self.get_logger().debug("🔵 BLUE LED ON (No person)")
                    
                elif self.current_status == "green":
                    self.led_pins['green'].value = True
                    self.get_logger().debug("🟢 GREEN LED ON (Unknown person)")
                    
            except Exception as e:
                self.get_logger().error(f"Error updating status LEDs: {str(e)}")
        else:
            # Simulation mode - just log
            emoji = {"red": "🔴", "blue": "🔵", "green": "🟢"}
            self.get_logger().info(
                f"[SIMULATED] {emoji.get(self.current_status, '⚫')} "
                f"{self.current_status.upper()} LED"
            )
    
    def _flash_yellow_led(self):
        """
        Flash yellow LED for configured duration (default 500ms).
        
        Uses threading.Timer to flash asynchronously without blocking.
        Cancels any existing flash timer to prevent overlap.
        """
        if not self.enabled:
            return
        
        # Cancel existing timer if still running
        if self.gesture_flash_timer and self.gesture_flash_timer.is_alive():
            self.gesture_flash_timer.cancel()
        
        if not self.simulate and self.led_pins:
            try:
                # Turn yellow LED ON
                self.led_pins['yellow'].value = True
                self.get_logger().debug("💛 YELLOW LED flash ON")
                
                # Schedule turning it OFF after duration
                def turn_off_yellow():
                    try:
                        self.led_pins['yellow'].value = False
                        self.get_logger().debug("💛 YELLOW LED flash OFF")
                    except Exception as e:
                        self.get_logger().error(f"Error turning off yellow LED: {e}")
                
                self.gesture_flash_timer = threading.Timer(
                    self.gesture_flash_duration, 
                    turn_off_yellow
                )
                self.gesture_flash_timer.start()
                
            except Exception as e:
                self.get_logger().error(f"Error flashing yellow LED: {str(e)}")
        else:
            # Simulation mode - just log
            self.get_logger().info(f"[SIMULATED] 💛 YELLOW LED flash ({self.gesture_flash_duration*1000:.0f}ms)")
    
    def _all_status_leds_off(self):
        """Turn all status LEDs off (does not affect yellow gesture LED)."""
        if not self.simulate and self.led_pins:
            try:
                self.led_pins['red'].value = False
                self.led_pins['blue'].value = False
                self.led_pins['green'].value = False
            except Exception as e:
                self.get_logger().error(f"Error turning off status LEDs: {e}")
    
    def _all_leds_off(self):
        """Turn ALL LEDs off (including yellow)."""
        if not self.simulate and self.led_pins:
            try:
                self.led_pins['red'].value = False
                self.led_pins['blue'].value = False
                self.led_pins['green'].value = False
                self.led_pins['yellow'].value = False
            except Exception as e:
                self.get_logger().error(f"Error turning off all LEDs: {e}")
    
    def destroy_node(self):
        """Clean up on node shutdown."""
        self.get_logger().info("Shutting down MCP23017 Status LED node...")
        
        # Cancel any active gesture flash timer
        if self.gesture_flash_timer and self.gesture_flash_timer.is_alive():
            self.gesture_flash_timer.cancel()
        
        # Turn all LEDs off
        self._all_leds_off()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MCP23017StatusLEDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MCP23017 Status LED Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

