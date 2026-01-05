#!/usr/bin/env python3
"""
RC Servo Driver for Jetson GPIO.

Controls standard RC servos via PWM on Jetson GPIO pins.
Uses RPi.GPIO library (compatible with Jetson via Jetson.GPIO).

Standard RC Servo:
- PWM Frequency: 50Hz (20ms period)
- Pulse Width: 1ms (0°) to 2ms (180°)
- Duty Cycle: 2.5% (0°) to 12.5% (180°)
"""

import time

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class ServoDriver:
    """
    RC Servo driver using GPIO PWM.
    
    Provides angle-based control for standard RC servos.
    Supports simulation mode when GPIO is unavailable.
    """
    
    # Standard RC servo PWM parameters
    PWM_FREQUENCY = 50  # Hz (20ms period)
    MIN_DUTY = 2.5      # Duty cycle for 0 degrees (~0.5ms pulse)
    MAX_DUTY = 12.5     # Duty cycle for 180 degrees (~2.5ms pulse)
    
    def __init__(
        self,
        gpio_pin: int,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        initial_angle: float = 90.0,
        simulate: bool = None
    ):
        """
        Initialize servo driver.
        
        Args:
            gpio_pin: BCM GPIO pin number for PWM signal
            min_angle: Minimum allowed angle (degrees)
            max_angle: Maximum allowed angle (degrees)
            initial_angle: Starting angle (degrees)
            simulate: Force simulation mode (auto-detect if None)
        """
        self.gpio_pin = gpio_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = initial_angle
        
        # Determine simulation mode
        if simulate is not None:
            self.simulate = simulate
        else:
            self.simulate = not HAS_GPIO
        
        self.pwm = None
        self._initialized = False
        
    def initialize(self) -> bool:
        """
        Initialize GPIO and start PWM.
        
        Returns:
            True if initialization successful, False otherwise.
        """
        if self._initialized:
            return True
            
        if self.simulate:
            print(f"[ServoDriver] Simulation mode - GPIO {self.gpio_pin}")
            self._initialized = True
            return True
        
        if not HAS_GPIO:
            print("[ServoDriver] RPi.GPIO not available - running in simulation mode")
            self.simulate = True
            self._initialized = True
            return True
        
        try:
            # Set up GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            
            # Create PWM instance
            self.pwm = GPIO.PWM(self.gpio_pin, self.PWM_FREQUENCY)
            
            # Start PWM at initial angle
            initial_duty = self._angle_to_duty(self.current_angle)
            self.pwm.start(initial_duty)
            
            self._initialized = True
            print(f"[ServoDriver] Initialized on GPIO {self.gpio_pin}, angle={self.current_angle}°")
            return True
            
        except Exception as e:
            print(f"[ServoDriver] Failed to initialize GPIO: {e}")
            self.simulate = True
            self._initialized = True
            return False
    
    def set_angle(self, angle: float):
        """
        Set servo to specified angle.
        
        Args:
            angle: Target angle in degrees (clamped to min/max)
        """
        if not self._initialized:
            self.initialize()
        
        # Clamp angle to valid range
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.current_angle = angle
        
        if self.simulate:
            # Simulation mode - just track angle
            return
        
        if self.pwm is not None:
            try:
                duty = self._angle_to_duty(angle)
                self.pwm.ChangeDutyCycle(duty)
            except Exception as e:
                print(f"[ServoDriver] Error setting angle: {e}")
    
    def get_angle(self) -> float:
        """Get current servo angle."""
        return self.current_angle
    
    def center(self):
        """Move servo to center position (90 degrees or midpoint of range)."""
        center_angle = (self.min_angle + self.max_angle) / 2.0
        self.set_angle(center_angle)
    
    def sweep_test(self, steps: int = 10, delay: float = 0.5):
        """
        Perform a sweep test from min to max angle.
        
        Args:
            steps: Number of steps in sweep
            delay: Delay between steps in seconds
        """
        if not self._initialized:
            self.initialize()
        
        print(f"[ServoDriver] Sweep test: {self.min_angle}° to {self.max_angle}°")
        
        # Sweep from min to max
        for i in range(steps + 1):
            angle = self.min_angle + (self.max_angle - self.min_angle) * i / steps
            self.set_angle(angle)
            print(f"  Angle: {angle:.1f}°")
            time.sleep(delay)
        
        # Return to center
        self.center()
        print(f"  Centered at: {self.current_angle:.1f}°")
    
    def _angle_to_duty(self, angle: float) -> float:
        """
        Convert angle to PWM duty cycle.
        
        Args:
            angle: Angle in degrees (0-180)
            
        Returns:
            Duty cycle percentage (2.5-12.5 for standard servo)
        """
        # Normalize angle to 0-180 range for duty calculation
        normalized = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        normalized = max(0.0, min(1.0, normalized))
        
        # Map to duty cycle range
        duty = self.MIN_DUTY + normalized * (self.MAX_DUTY - self.MIN_DUTY)
        return duty
    
    def cleanup(self):
        """Stop PWM and cleanup GPIO."""
        if self.pwm is not None:
            try:
                self.pwm.stop()
            except Exception:
                pass
        
        if not self.simulate and HAS_GPIO:
            try:
                GPIO.cleanup(self.gpio_pin)
            except Exception:
                pass
        
        self._initialized = False
        print(f"[ServoDriver] Cleanup complete for GPIO {self.gpio_pin}")
    
    def __del__(self):
        """Destructor - ensure cleanup."""
        self.cleanup()


# Standalone test
if __name__ == '__main__':
    print("Servo Driver Test")
    print("=" * 40)
    
    # Create servo on GPIO 13 (Pin 33)
    servo = ServoDriver(
        gpio_pin=13,
        min_angle=60.0,
        max_angle=120.0,
        initial_angle=90.0
    )
    
    if servo.initialize():
        print("\nRunning sweep test...")
        servo.sweep_test(steps=6, delay=0.3)
        
        print("\nManual angle test...")
        for angle in [70, 90, 110, 90]:
            print(f"Setting angle to {angle}°")
            servo.set_angle(angle)
            time.sleep(0.5)
        
        servo.cleanup()
    else:
        print("Failed to initialize servo")

