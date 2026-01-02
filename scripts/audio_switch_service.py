#!/usr/bin/env python3
"""
R2D2 Audio Switch Service
Monitors GPIO switch and routes audio to Bluetooth OR PAM8403
"""
import Jetson.GPIO as GPIO
import subprocess
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

SWITCH_PIN = 17
BLUETOOTH_SINK = "bluez_sink.28_54_71_BB_C6_53.a2dp_sink"
PAM8403_SINK = "alsa_output.platform-sound.stereo-fallback"

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def get_output_mode():
    """Returns 'bluetooth' or 'pam8403' based on switch"""
    return 'bluetooth' if GPIO.input(SWITCH_PIN) == GPIO.HIGH else 'pam8403'

def set_audio_sink(mode):
    """Set PulseAudio sink based on mode"""
    sink = BLUETOOTH_SINK if mode == 'bluetooth' else PAM8403_SINK
    try:
        subprocess.run(['pactl', 'set-default-sink', sink], 
                      check=True, capture_output=True,
                      env={'XDG_RUNTIME_DIR': '/run/user/1000',
                           'PULSE_SERVER': 'unix:/run/user/1000/pulse/native'})
        logger.info(f"Audio output: {mode.upper()} ({sink})")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to set sink: {e}")
        return False

def main():
    setup_gpio()
    logger.info("R2D2 Audio Switch Service Started")
    logger.info("Switch UP=Bluetooth, Switch DOWN=PAM8403")
    
    last_mode = None
    
    try:
        while True:
            current_mode = get_output_mode()
            
            if current_mode != last_mode:
                logger.info(f"Switch changed → {current_mode.upper()}")
                set_audio_sink(current_mode)
                last_mode = current_mode
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        logger.info("Service stopped")
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()

