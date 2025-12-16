#!/usr/bin/env python3
"""
R2D2 Power Button - FIXED VERSION with Audio

Button 1 (Pin 32 + Pin 39 GND):
  - Single Press = Play R2-D2 sound → Shutdown (shutdown -h now)
  
FIXES:
- Handles pin starting in LOW state
- Waits for proper HIGH state before monitoring
- Only triggers on complete press→release cycle

FEATURES:
- Plays MP3 audio file before shutdown (system default volume)
- Audio file: /home/severin/Voicy_R2-D2 - 3.mp3
- Uses ffplay with -nodisp -autoexit flags
- Graceful error handling (shutdown proceeds even if audio fails)
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
STARTUP_WAIT_TIME = 2.0  # Wait up to 2 seconds for pin to stabilize

# Logging
LOG_FILE = "/var/log/r2d2_power_button.log"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

def setup_logging():
    """Configure logging."""
    logger = logging.getLogger("r2d2-powerbutton")
    logger.setLevel(logging.DEBUG)
    
    # Remove existing handlers to avoid duplicates
    logger.handlers = []
    
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
    """Initialize GPIO and wait for stable HIGH state."""
    try:
        GPIO.setmode(GPIO.BOARD)
        # Note: Jetson.GPIO may ignore pull_up_down, but we set it anyway
        GPIO.setup(BUTTON_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Wait for GPIO to stabilize
        time.sleep(0.2)
        
        # Read initial state
        initial_state = GPIO.input(BUTTON_GPIO_PIN)
        logger.info(f"✓ GPIO initialized (Pin {BUTTON_GPIO_PIN})")
        logger.info(f"  Initial state: {initial_state} ({'HIGH' if initial_state == GPIO.HIGH else 'LOW'})")
        
        # If pin is LOW at startup, wait for it to go HIGH
        if initial_state == GPIO.LOW:
            logger.warning("  Pin is LOW at startup - waiting for button to be released...")
            start_time = time.time()
            while GPIO.input(BUTTON_GPIO_PIN) == GPIO.LOW:
                elapsed = time.time() - start_time
                if elapsed > STARTUP_WAIT_TIME:
                    logger.error(f"  ERROR: Pin still LOW after {STARTUP_WAIT_TIME}s")
                    logger.error("  Possible causes:")
                    logger.error("    1. Button is stuck pressed")
                    logger.error("    2. Wiring issue (check Pin 32 + Pin 39 GND)")
                    logger.error("    3. Pull-up not working (Jetson.GPIO limitation)")
                    logger.error("    4. Need external pull-up resistor")
                    return False
                time.sleep(0.1)
            
            logger.info(f"  Pin went HIGH after {time.time() - start_time:.2f}s - ready to monitor")
        else:
            logger.info("  Pin is HIGH - ready to monitor")
        
        # Verify pin is HIGH before starting
        final_state = GPIO.input(BUTTON_GPIO_PIN)
        if final_state != GPIO.HIGH:
            logger.error(f"  ERROR: Pin is not HIGH after initialization (state: {final_state})")
            return False
        
        logger.info("  ✓ GPIO ready - monitoring for button presses")
        return True
        
    except Exception as e:
        logger.error(f"Failed to initialize GPIO: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return False

def cleanup_gpio():
    """Clean up GPIO."""
    try:
        GPIO.cleanup()
        logger.info("✓ GPIO cleaned up")
    except Exception as e:
        logger.error(f"Error during GPIO cleanup: {e}")

def play_shutdown_sound():
    """Play R2-D2 shutdown sound before shutting down."""
    mp3_file = "/home/severin/Voicy_R2-D2 - 3.mp3"
    timeout = 30  # Maximum playback time in seconds
    
    try:
        logger.info(f"Playing shutdown sound: {mp3_file}")
        
        # Check if file exists
        import os
        if not os.path.exists(mp3_file):
            logger.warning(f"MP3 file not found: {mp3_file} - proceeding with shutdown")
            return False
        
        # Play audio using ffplay with system default volume
        # -nodisp: No video window (audio only)
        # -autoexit: Exit when playback completes
        # -loglevel quiet: Suppress output
        result = subprocess.run(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", mp3_file],
            timeout=timeout,
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            logger.info("✓ Shutdown sound played successfully")
            return True
        else:
            logger.warning(f"Audio playback returned non-zero exit code: {result.returncode}")
            if result.stderr:
                logger.warning(f"ffplay stderr: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        logger.warning(f"Audio playback timed out after {timeout}s - proceeding with shutdown")
        return False
    except FileNotFoundError:
        logger.warning("ffplay not found - proceeding with shutdown without audio")
        return False
    except Exception as e:
        logger.warning(f"Error playing shutdown sound: {e} - proceeding with shutdown")
        import traceback
        logger.debug(traceback.format_exc())
        return False

def execute_shutdown():
    """Execute graceful shutdown."""
    try:
        logger.warning("="*70)
        logger.warning("ACTION: Shutting down system...")
        logger.warning("="*70)
        
        # Play shutdown sound first
        play_shutdown_sound()
        
        # Service runs as root, so no sudo needed
        result = subprocess.run(["shutdown", "-h", "now"], timeout=5, capture_output=True, text=True)
        logger.info(f"✓ Shutdown command issued (return code: {result.returncode})")
        if result.stderr:
            logger.warning(f"Shutdown stderr: {result.stderr}")
    except subprocess.TimeoutExpired:
        logger.error("Shutdown command timed out!")
    except Exception as e:
        logger.error(f"Shutdown error: {e}")
        import traceback
        logger.error(traceback.format_exc())

def button_monitor():
    """Monitor button for complete press→release cycles."""
    # Start monitoring from HIGH state (button not pressed)
    current_state = GPIO.input(BUTTON_GPIO_PIN)
    if current_state != GPIO.HIGH:
        logger.error(f"Cannot start monitoring - pin is not HIGH (state: {current_state})")
        return
    
    logger.info(f"Button monitor active (starting from HIGH state)")
    
    press_start_time = 0
    last_state = GPIO.HIGH
    last_state_change_time = time.time()
    
    try:
        while True:
            current_state = GPIO.input(BUTTON_GPIO_PIN)
            current_time = time.time()
            
            # Check for state change
            if current_state != last_state:
                # State changed - reset debounce timer
                last_state = current_state
                last_state_change_time = current_time
            else:
                # State is stable - check if it's been stable long enough
                stable_duration = current_time - last_state_change_time
                
                if stable_duration > DEBOUNCE_TIME:
                    # State has been stable - process the transition
                    if current_state == GPIO.LOW and press_start_time == 0:
                        # Button pressed (HIGH → LOW transition, stable)
                        press_start_time = current_time
                        logger.info("Button PRESSED detected")
                    elif current_state == GPIO.HIGH and press_start_time > 0:
                        # Button released (LOW → HIGH transition, stable)
                        press_duration = current_time - press_start_time
                        logger.info(f"Button RELEASED detected (duration: {press_duration:.2f}s)")
                        
                        # Trigger shutdown if press was long enough
                        if press_duration > 0.1:
                            logger.warning(">>> Initiating shutdown... <<<")
                            execute_shutdown()
                        else:
                            logger.debug(f"Press too short ({press_duration:.2f}s) - ignored")
                        
                        press_start_time = 0  # Reset for next press
            
            time.sleep(POLLING_INTERVAL)
    
    except KeyboardInterrupt:
        logger.info("Button monitor stopped by user")
    except Exception as e:
        logger.error(f"Error in button monitor: {e}")
        import traceback
        logger.error(traceback.format_exc())

def signal_handler(signum, frame):
    """Handle termination signals."""
    logger.info(f"Received signal {signum}, shutting down...")
    cleanup_gpio()
    sys.exit(0)

def main():
    """Main entry point."""
    logger.info("="*70)
    logger.info("R2D2 Power Button Handler - FIXED VERSION")
    logger.info("="*70)
    logger.info(f"Button 1 (Pin 32): Single press = Shutdown")
    logger.info(f"Button 2 (J42 Pin 4 + Pin 1): Short pins = Wake/Boot")
    logger.info("="*70)
    
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    if not setup_gpio():
        logger.error("GPIO setup failed - exiting")
        sys.exit(1)
    
    try:
        button_monitor()
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()

