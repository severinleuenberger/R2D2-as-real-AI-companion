#!/usr/bin/env python3
"""
LED Control Interface

Architecture for LED support that can be easily extended.
Currently supports:
  - Text status display (console)
  - Future: GPIO-based RGB LED
  - Future: Network LED (HTTP requests)
  - Future: Serial-based LED controller

Design allows hot-swapping different LED backends without changing main service.
"""

from pathlib import Path
from datetime import datetime
import json


class LEDInterface:
    """Base interface for LED control."""
    
    def set_recognized(self, person_name):
        """Set LED to 'recognized' state."""
        raise NotImplementedError
    
    def set_unrecognized(self):
        """Set LED to 'unrecognized' state."""
        raise NotImplementedError
    
    def set_error(self):
        """Set LED to 'error' state."""
        raise NotImplementedError


class TextLED(LEDInterface):
    """Text-based LED display (console output)."""
    
    def set_recognized(self, person_name):
        """Display recognized person."""
        print(f"\r✅ RECOGNIZED: {person_name.upper():<20}", end='', flush=True)
    
    def set_unrecognized(self):
        """Display unrecognized."""
        print(f"\r❌ No one recognized             ", end='', flush=True)
    
    def set_error(self):
        """Display error."""
        print(f"\r⚠️  ERROR                        ", end='', flush=True)


class GPIOLEDController(LEDInterface):
    """
    GPIO-based RGB LED controller.
    
    Requires:
      - RPi.GPIO or board-specific GPIO library
      - 3 GPIO pins for R, G, B
      - Common-cathode RGB LED
    
    States:
      - Recognized: Green
      - Unrecognized: Red
      - Error: Yellow (Red + Green)
    """
    
    def __init__(self, red_pin=17, green_pin=27, blue_pin=22):
        """
        Initialize GPIO LED controller.
        
        Args:
            red_pin: GPIO pin for red LED
            green_pin: GPIO pin for green LED
            blue_pin: GPIO pin for blue LED
        """
        self.red_pin = red_pin
        self.green_pin = green_pin
        self.blue_pin = blue_pin
        
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(red_pin, GPIO.OUT)
            GPIO.setup(green_pin, GPIO.OUT)
            GPIO.setup(blue_pin, GPIO.OUT)
            print(f"✓ GPIO LED initialized (R:{red_pin}, G:{green_pin}, B:{blue_pin})")
        except ImportError:
            print("⚠️  RPi.GPIO not available - GPIO LED support disabled")
            self.GPIO = None
    
    def _set_color(self, red, green, blue):
        """Set LED color."""
        if not self.GPIO:
            return
        
        self.GPIO.output(self.red_pin, red)
        self.GPIO.output(self.green_pin, green)
        self.GPIO.output(self.blue_pin, blue)
    
    def set_recognized(self, person_name):
        """Green LED for recognized."""
        self._set_color(False, True, False)  # Green
        print(f"✅ RECOGNIZED: {person_name.upper()}")
    
    def set_unrecognized(self):
        """Red LED for unrecognized."""
        self._set_color(True, False, False)  # Red
        print(f"❌ No one recognized")
    
    def set_error(self):
        """Yellow LED for error."""
        self._set_color(True, True, False)  # Yellow
        print(f"⚠️  ERROR")
    
    def cleanup(self):
        """Cleanup GPIO."""
        if self.GPIO:
            self.GPIO.cleanup()


class HTTPLEDController(LEDInterface):
    """
    Network-based LED controller via HTTP.
    
    Calls HTTP endpoint with status:
      - GET /api/led?status=recognized&person=severin
      - GET /api/led?status=unrecognized
      - GET /api/led?status=error
    """
    
    def __init__(self, base_url):
        """
        Initialize HTTP LED controller.
        
        Args:
            base_url: Base URL of LED controller (e.g., http://192.168.1.100:5000)
        """
        self.base_url = base_url.rstrip('/')
        print(f"✓ HTTP LED initialized ({base_url})")
    
    def _send_request(self, status, person=None):
        """Send HTTP request to LED controller."""
        try:
            import requests
            url = f"{self.base_url}/api/led"
            params = {'status': status}
            if person:
                params['person'] = person
            
            requests.get(url, params=params, timeout=2)
        except Exception as e:
            print(f"⚠️  HTTP LED request failed: {e}")
    
    def set_recognized(self, person_name):
        """Send recognized status."""
        self._send_request('recognized', person_name)
        print(f"✅ RECOGNIZED: {person_name.upper()}")
    
    def set_unrecognized(self):
        """Send unrecognized status."""
        self._send_request('unrecognized')
        print(f"❌ No one recognized")
    
    def set_error(self):
        """Send error status."""
        self._send_request('error')
        print(f"⚠️  ERROR")


def create_led_controller(led_type='text', **kwargs):
    """
    Factory function to create LED controller.
    
    Args:
        led_type: 'text', 'gpio', or 'http'
        **kwargs: Additional arguments for the controller
    
    Returns:
        LEDInterface instance
    
    Examples:
        # Text display (default)
        led = create_led_controller('text')
        
        # GPIO-based RGB LED
        led = create_led_controller('gpio', red_pin=17, green_pin=27, blue_pin=22)
        
        # Network-based LED
        led = create_led_controller('http', base_url='http://192.168.1.100:5000')
    """
    if led_type == 'gpio':
        return GPIOLEDController(**kwargs)
    elif led_type == 'http':
        return HTTPLEDController(**kwargs)
    else:
        return TextLED()


if __name__ == "__main__":
    # Demo
    print("LED Control System - Demo")
    print()
    
    # Text LED
    print("Text LED:")
    led = create_led_controller('text')
    led.set_recognized('severin')
    print()
    led.set_unrecognized()
    print()
    led.set_error()
    print()
    print()
    
    print("✓ LED architecture ready for integration")
