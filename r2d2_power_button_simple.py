#!/usr/bin/env python3
"""
R2D2 Power Button - SIMPLE VERSION

Button 1 (Pin 32 + Pin 39 GND):
  - Single Press = Shutdown (shutdown -h now)

Button 2 (J42 Pin 4 + J42 Pin 1 GND):
  - Shorting these pins = Wake/Boot the system

That's it. Simple and focused.
"""

import Jetson.GPIO as GPIO
import time
import subprocess
import logging
import sys
import signal

# Configuration
BUTTON_GPIO_PIN = 32  # Pin 32 on 40-pin header
DEBOUNCE_TIME = 0.1
POLLING_INTERVAL = 0.02

# Logging
LOG_FILE = "/var/log/r2d2_power_button.log"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

def setup_logging():
    """Configure logging."""
    logger = logging.getLogger("r2d2-powerbutton")
    logger.setLevel(logging.DEBUG)
    
    # File handler
    try:
        file_handler = logging.FileHandler(LOG_FILE)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
        logger.addHandler(file_handler)
    except PermissionError:
        print(f"Warning: Cannot write to {LOG_FILE}")
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
    logger.addHandler(console_handler)
    
    return logger

logger = setup_logging()

def setup_gpio():
    """Initialize GPIO."""
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BUTTON_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        logger.info(f"✓ GPIO initialized (Pin {BUTTON_GPIO_PIN})")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize GPIO: {e}")
        return False

def cleanup_gpio():
    """Clean up GPIO."""
    try:
        GPIO.cleanup()
        logger.info("✓ GPIO cleaned up")
    except Exception as e:
        logger.error(f"Error during GPIO cleanup: {e}")

def execute_shutdown():
    """Execute graceful shutdown."""
    try:
        logger.info("ACTION: Shutting down system...")
        subprocess.run(["sudo", "shutdown", "-h", "now"], timeout=5)
        logger.info("✓ Shutdown command issued")
    except Exception as e:
        logger.error(f"Shutdown error: {e}")

def button_monitor():
    """Monitor button for single presses."""
    last_state = GPIO.HIGH
    stable_time = 0
    stable_state = GPIO.HIGH
    press_start = 0
    
    try:
        logger.info("Button monitor active")
        
        while True:
            current_state = GPIO.input(BUTTON_GPIO_PIN)
            current_time = time.time()
            
            # Debouncing
            if current_state == last_state:
                if current_time - stable_time > DEBOUNCE_TIME:
                    if stable_state != current_state:
                        stable_state = current_state
                        
                        if stable_state == GPIO.LOW:
                            # Button pressed
                            press_start = current_time
                            logger.info("Button pressed")
                        else:
                            # Button released
                            press_duration = current_time - press_start
                            logger.info(f"Button released (duration: {press_duration:.2f}s)")
                            
                            # Any press = trigger shutdown
                            if press_duration > 0.1:  # Debounce minimum
                                logger.info("Initiating shutdown...")
                                execute_shutdown()
            else:
                last_state = current_state
                stable_time = current_time
            
            time.sleep(POLLING_INTERVAL)
    
    except KeyboardInterrupt:
        logger.info("Button monitor stopped by user")
    except Exception as e:
        logger.error(f"Error in button monitor: {e}")

def signal_handler(signum, frame):
    """Handle termination signals."""
    logger.info(f"Received signal {signum}, shutting down...")
    cleanup_gpio()
    sys.exit(0)

def main():
    """Main entry point."""
    logger.info("="*70)
    logger.info("R2D2 Power Button Handler - SIMPLE VERSION")
    logger.info("="*70)
    logger.info(f"Button 1 (Pin 32): Single press = Shutdown")
    logger.info(f"Button 2 (J42 Pin 4 + Pin 1): Short pins = Wake/Boot")
    logger.info("="*70)
    
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    if not setup_gpio():
        sys.exit(1)
    
    try:
        button_monitor()
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
