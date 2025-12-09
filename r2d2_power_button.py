#!/usr/bin/env python3
"""
R2D2 Power Button Handler - FINAL WORKING VERSION

BUTTON 1 (Software Control):
  - Single Press = Low-Power Mode (nvpmodel -m 2)
  - Double Press = Graceful Shutdown (shutdown -h now)
  
  Wiring:
    - Pin 32 (GPIO09, 40-pin header) ← Button signal
    - Pin 39 (GND, 40-pin header) ← Ground

BUTTON 2 (Hardware Power Control - separate button):
  - Single Press = Wake from low-power OR Boot from shutdown
  
  Wiring:
    - J42 Pin 2 (PWR_BTN) ← Button signal
    - J42 Pin 1 (GND) ← Ground
  
  Note: This button is hardware-controlled and works regardless of system state
"""

import Jetson.GPIO as GPIO
import time
import subprocess
import logging
import sys
import signal
from datetime import datetime

# ============================================================================
# CONFIGURATION
# ============================================================================

BUTTON_GPIO_PIN = 32  # Physical pin 32 on 40-pin header (GPIO09, has pull-up resistor)
DEBOUNCE_TIME = 0.1  # 100ms debounce
DOUBLE_PRESS_WINDOW = 1.0  # 1000ms window for double-press detection (was 500ms)
LONG_PRESS_THRESHOLD = 2.0  # 2s threshold for long press (counts as single)
POLLING_INTERVAL = 0.02  # 20ms polling interval

# Logging configuration
LOG_FILE = "/var/log/r2d2_power_button.log"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

# ============================================================================
# LOGGING SETUP
# ============================================================================

def setup_logging():
    """Configure logging to both file and journalctl."""
    logger = logging.getLogger("r2d2-powerbutton")
    logger.setLevel(logging.DEBUG)
    
    # File handler
    try:
        file_handler = logging.FileHandler(LOG_FILE)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
        logger.addHandler(file_handler)
    except PermissionError:
        print(f"Warning: Cannot write to {LOG_FILE}, using stdout only")
    
    # Console handler (goes to journalctl via systemd)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
    logger.addHandler(console_handler)
    
    return logger

logger = setup_logging()

# ============================================================================
# BUTTON STATE MACHINE
# ============================================================================

class PowerButtonHandler:
    """Manages power button state machine and action execution."""
    
    def __init__(self):
        self.last_press_time = 0
        self.press_count = 0
        self.in_double_press_window = False
        self.shutdown_in_progress = False
        self.button_pressed = False
        self.timeout_thread = None
        
    def on_button_press(self):
        """Handle button press event (falling edge)."""
        current_time = time.time()
        time_since_last = current_time - self.last_press_time
        
        logger.debug(f"Button pressed. Last press: {time_since_last:.3f}s ago")
        
        # Check if we're within the double-press window from the FIRST press
        if self.in_double_press_window and time_since_last < DOUBLE_PRESS_WINDOW:
            # Second press detected within window!
            self.press_count += 1
            logger.info(f"Press count: {self.press_count}")
            
            if self.press_count >= 2:
                # Double press confirmed!
                self.in_double_press_window = False
                logger.info("Double-press confirmed!")
                self.execute_shutdown()
                self.reset_state()
                return
        else:
            # Either new sequence or outside window
            if not self.in_double_press_window or time_since_last >= DOUBLE_PRESS_WINDOW:
                # Starting fresh or window expired - execute pending single press if exists
                if self.in_double_press_window and self.press_count == 1:
                    logger.info("Previous single press window expired.")
                    self.execute_low_power()
                
                # Reset for new sequence
                self.press_count = 1
                self.in_double_press_window = True
                self.last_press_time = current_time
                logger.info("Single press detected, waiting for second press...")
                
                # Schedule the timeout to execute single-press action
                self._schedule_single_press_timeout(current_time)
                return
        
        self.last_press_time = current_time
    
    def _schedule_single_press_timeout(self, press_time):
        """
        Wait for DOUBLE_PRESS_WINDOW to expire.
        If no second press occurs, execute single-press action.
        """
        def timeout_check():
            time.sleep(DOUBLE_PRESS_WINDOW + 0.05)
            current_time = time.time()
            
            # Check if we're STILL in the window AND still have press_count == 1
            if self.in_double_press_window and self.press_count == 1:
                logger.info("Double-press window expired. Single press confirmed.")
                self.in_double_press_window = False
                self.execute_low_power()
                self.reset_state()
        
        # Run timeout check in background (daemon thread)
        import threading
        self.timeout_thread = threading.Thread(target=timeout_check, daemon=True)
        self.timeout_thread.start()
    
    def execute_low_power(self):
        """Execute low-power mode (nvpmodel -m 2)."""
        if self.shutdown_in_progress:
            logger.warning("Shutdown already in progress, ignoring low-power request")
            return
        
        try:
            logger.info("ACTION: Entering low-power mode (nvpmodel -m 2)")
            result = subprocess.run(
                ["sudo", "nvpmodel", "-m", "2"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                logger.info("✓ Low-power mode activated successfully")
            else:
                logger.error(f"nvpmodel failed: {result.stderr}")
        except FileNotFoundError:
            logger.error("nvpmodel not found! Is it installed?")
        except subprocess.TimeoutExpired:
            logger.error("nvpmodel command timed out")
        except Exception as e:
            logger.error(f"Error executing low-power mode: {e}")
    
    def execute_shutdown(self):
        """Execute graceful shutdown (shutdown -h now)."""
        if self.shutdown_in_progress:
            logger.warning("Shutdown already in progress, ignoring additional request")
            return
        
        self.shutdown_in_progress = True
        
        try:
            logger.info("ACTION: Initiating graceful shutdown (shutdown -h now)")
            result = subprocess.run(
                ["sudo", "shutdown", "-h", "now"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                logger.info("✓ Shutdown command issued successfully")
            else:
                logger.error(f"shutdown failed: {result.stderr}")
                self.shutdown_in_progress = False
        except FileNotFoundError:
            logger.error("shutdown command not found!")
            self.shutdown_in_progress = False
        except subprocess.TimeoutExpired:
            logger.error("shutdown command timed out")
            self.shutdown_in_progress = False
        except Exception as e:
            logger.error(f"Error executing shutdown: {e}")
            self.shutdown_in_progress = False
    
    def reset_state(self):
        """Reset button state machine."""
        self.press_count = 0
        self.in_double_press_window = False
        self.button_pressed = False
    
    def on_button_release(self):
        """Handle button release event (rising edge)."""
        logger.debug("Button released")

# ============================================================================
# GPIO SETUP AND MONITORING
# ============================================================================

def setup_gpio():
    """Initialize GPIO and set up button pin."""
    try:
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbers
        GPIO.setup(BUTTON_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        logger.info(f"✓ GPIO initialized (BOARD mode, Pin {BUTTON_GPIO_PIN} with pull-up)")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize GPIO: {e}")
        return False

def cleanup_gpio():
    """Clean up GPIO resources."""
    try:
        GPIO.cleanup()
        logger.info("✓ GPIO cleaned up")
    except Exception as e:
        logger.error(f"Error during GPIO cleanup: {e}")

def debounce_and_detect(handler):
    """
    Monitor button pin with debouncing.
    Detects falling edge (press) and rising edge (release).
    """
    last_state = GPIO.HIGH
    stable_time = 0
    stable_state = GPIO.HIGH
    
    try:
        while True:
            current_state = GPIO.input(BUTTON_GPIO_PIN)
            current_time = time.time()
            
            # Debouncing logic
            if current_state == last_state:
                if current_time - stable_time > DEBOUNCE_TIME:
                    # State is stable
                    if stable_state != current_state:
                        # State changed after debounce
                        stable_state = current_state
                        
                        if stable_state == GPIO.LOW:
                            # Button pressed (falling edge)
                            handler.on_button_press()
                        else:
                            # Button released (rising edge)
                            handler.on_button_release()
            else:
                # State changed, reset timer
                last_state = current_state
                stable_time = current_time
            
            time.sleep(POLLING_INTERVAL)  # Polling interval (20ms)
    
    except KeyboardInterrupt:
        logger.info("Button monitor interrupted by user")
    except Exception as e:
        logger.error(f"Error in button monitor: {e}")

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

def signal_handler(signum, frame):
    """Handle termination signals gracefully."""
    logger.info(f"Received signal {signum}, shutting down...")
    cleanup_gpio()
    sys.exit(0)

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main entry point."""
    logger.info("="*70)
    logger.info("R2D2 Power Button Handler starting")
    logger.info("="*70)
    logger.info(f"Configuration:")
    logger.info(f"  GPIO Pin: {BUTTON_GPIO_PIN} (40-pin header, has pull-up)")
    logger.info(f"  Debounce: {DEBOUNCE_TIME*1000:.0f}ms")
    logger.info(f"  Double-press window: {DOUBLE_PRESS_WINDOW*1000:.0f}ms")
    logger.info(f"  Long-press threshold: {LONG_PRESS_THRESHOLD:.1f}s")
    logger.info(f"  Log file: {LOG_FILE}")
    logger.info("="*70)
    
    # Set up signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize GPIO
    if not setup_gpio():
        logger.error("Failed to initialize GPIO, exiting")
        sys.exit(1)
    
    # Create button handler
    handler = PowerButtonHandler()
    
    try:
        logger.info("Button monitor active, waiting for presses...")
        debounce_and_detect(handler)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)
    finally:
        cleanup_gpio()
        logger.info("R2D2 Power Button Handler stopped")

if __name__ == "__main__":
    main()
