#!/usr/bin/env python3
"""
Simple GPIO toggle test to verify pins 7 and 11 work.
Use a multimeter to measure the voltage on these pins.
"""

import time
import Jetson.GPIO as GPIO

PIN_7 = 7
PIN_11 = 11

def main():
    print("=" * 50)
    print("GPIO Toggle Test")
    print("=" * 50)
    print("\nThis will toggle Pin 7 and Pin 11.")
    print("Use a multimeter to verify voltage changes.\n")
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    GPIO.setup(PIN_7, GPIO.OUT)
    GPIO.setup(PIN_11, GPIO.OUT)
    
    try:
        # Test 1: Both HIGH
        print("TEST 1: Both pins HIGH")
        print("  Pin 7  = HIGH (should read ~3.3V)")
        print("  Pin 11 = HIGH (should read ~3.3V)")
        GPIO.output(PIN_7, GPIO.HIGH)
        GPIO.output(PIN_11, GPIO.HIGH)
        input("  Measure now, then press Enter...")
        
        # Test 2: Both LOW
        print("\nTEST 2: Both pins LOW")
        print("  Pin 7  = LOW (should read ~0V)")
        print("  Pin 11 = LOW (should read ~0V)")
        GPIO.output(PIN_7, GPIO.LOW)
        GPIO.output(PIN_11, GPIO.LOW)
        input("  Measure now, then press Enter...")
        
        # Test 3: Pin 7 HIGH, Pin 11 LOW
        print("\nTEST 3: Pin 7 HIGH, Pin 11 LOW")
        print("  Pin 7  = HIGH (should read ~3.3V)")
        print("  Pin 11 = LOW (should read ~0V)")
        GPIO.output(PIN_7, GPIO.HIGH)
        GPIO.output(PIN_11, GPIO.LOW)
        input("  Measure now, then press Enter...")
        
        # Test 4: Pin 7 LOW, Pin 11 HIGH
        print("\nTEST 4: Pin 7 LOW, Pin 11 HIGH")
        print("  Pin 7  = LOW (should read ~0V)")
        print("  Pin 11 = HIGH (should read ~3.3V)")
        GPIO.output(PIN_7, GPIO.LOW)
        GPIO.output(PIN_11, GPIO.HIGH)
        input("  Measure now, then press Enter...")
        
        # Test 5: Fast toggle (visual with LED if connected)
        print("\nTEST 5: Fast toggle (5 seconds)")
        print("  If you connect an LED to Pin 7, it should blink.")
        for i in range(50):
            GPIO.output(PIN_7, GPIO.HIGH)
            GPIO.output(PIN_11, GPIO.HIGH)
            time.sleep(0.05)
            GPIO.output(PIN_7, GPIO.LOW)
            GPIO.output(PIN_11, GPIO.LOW)
            time.sleep(0.05)
        
        print("\n✓ Test complete!")
        
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()


