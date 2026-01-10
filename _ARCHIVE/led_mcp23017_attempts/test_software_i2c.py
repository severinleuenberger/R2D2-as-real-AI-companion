#!/usr/bin/env python3
"""
Software I2C Test for MCP23017 on Jetson AGX Orin

Since hardware I2C pins (3/5) are not properly muxed, we use software I2C
on regular GPIO pins.

Wiring:
  - Pin 7  -> SCL (MCP23017 SCK)
  - Pin 11 -> SDA (MCP23017 SDA)
  - Pin 1  -> VCC (3.3V)
  - Pin 6  -> GND
"""

import time
import Jetson.GPIO as GPIO

# Pin assignments (BOARD numbering)
SCL_PIN = 7
SDA_PIN = 11

# MCP23017 address (A0=A1=A2=0 -> 0x20)
MCP23017_ADDR = 0x20

# MCP23017 registers
IODIRA = 0x00  # I/O Direction Register A
IODIRB = 0x01  # I/O Direction Register B
GPIOA = 0x12   # GPIO Register A
GPIOB = 0x13   # GPIO Register B

# I2C timing (microseconds)
I2C_DELAY = 0.00001  # 10us = ~50kHz


class SoftwareI2C:
    """Bit-banged I2C implementation using Jetson GPIO"""
    
    def __init__(self, scl_pin, sda_pin):
        self.scl = scl_pin
        self.sda = sda_pin
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        # Start with both lines high (idle state)
        GPIO.setup(self.scl, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.sda, GPIO.OUT, initial=GPIO.HIGH)
        time.sleep(0.001)
    
    def _delay(self):
        time.sleep(I2C_DELAY)
    
    def _scl_high(self):
        GPIO.output(self.scl, GPIO.HIGH)
        self._delay()
    
    def _scl_low(self):
        GPIO.output(self.scl, GPIO.LOW)
        self._delay()
    
    def _sda_high(self):
        GPIO.setup(self.sda, GPIO.OUT)
        GPIO.output(self.sda, GPIO.HIGH)
        self._delay()
    
    def _sda_low(self):
        GPIO.setup(self.sda, GPIO.OUT)
        GPIO.output(self.sda, GPIO.LOW)
        self._delay()
    
    def _sda_read(self):
        GPIO.setup(self.sda, GPIO.IN)
        self._delay()
        return GPIO.input(self.sda)
    
    def _start(self):
        """I2C Start condition: SDA goes low while SCL is high"""
        self._sda_high()
        self._scl_high()
        self._sda_low()
        self._scl_low()
    
    def _stop(self):
        """I2C Stop condition: SDA goes high while SCL is high"""
        self._sda_low()
        self._scl_high()
        self._sda_high()
    
    def _write_byte(self, byte):
        """Write a byte, return ACK (True=ACK received)"""
        for i in range(8):
            if byte & 0x80:
                self._sda_high()
            else:
                self._sda_low()
            self._scl_high()
            self._scl_low()
            byte <<= 1
        
        # Read ACK
        ack = self._sda_read()
        self._scl_high()
        self._scl_low()
        GPIO.setup(self.sda, GPIO.OUT)
        
        return ack == GPIO.LOW  # ACK is low
    
    def _read_byte(self, ack=True):
        """Read a byte, send ACK/NACK"""
        byte = 0
        GPIO.setup(self.sda, GPIO.IN)
        
        for i in range(8):
            self._scl_high()
            byte = (byte << 1) | (1 if GPIO.input(self.sda) else 0)
            self._scl_low()
        
        # Send ACK/NACK
        GPIO.setup(self.sda, GPIO.OUT)
        if ack:
            self._sda_low()  # ACK
        else:
            self._sda_high()  # NACK
        self._scl_high()
        self._scl_low()
        
        return byte
    
    def write_register(self, addr, reg, value):
        """Write a value to a register"""
        self._start()
        if not self._write_byte(addr << 1):  # Write mode
            self._stop()
            return False
        if not self._write_byte(reg):
            self._stop()
            return False
        if not self._write_byte(value):
            self._stop()
            return False
        self._stop()
        return True
    
    def read_register(self, addr, reg):
        """Read a value from a register"""
        self._start()
        if not self._write_byte(addr << 1):  # Write mode
            self._stop()
            return None
        if not self._write_byte(reg):
            self._stop()
            return None
        
        self._start()  # Repeated start
        if not self._write_byte((addr << 1) | 1):  # Read mode
            self._stop()
            return None
        
        value = self._read_byte(ack=False)
        self._stop()
        return value
    
    def scan(self):
        """Scan for I2C devices"""
        devices = []
        for addr in range(0x08, 0x78):
            self._start()
            if self._write_byte(addr << 1):
                devices.append(addr)
            self._stop()
            time.sleep(0.001)
        return devices
    
    def cleanup(self):
        GPIO.cleanup()


def main():
    print("=" * 60)
    print("Software I2C Test for MCP23017")
    print("=" * 60)
    print(f"\nUsing pins: SCL={SCL_PIN}, SDA={SDA_PIN}")
    print("\nMake sure MCP23017 is wired:")
    print(f"  - Pin {SCL_PIN} -> SCL (MCP23017 SCK)")
    print(f"  - Pin {SDA_PIN} -> SDA (MCP23017 SDA)")
    print("  - Pin 1   -> VCC (3.3V)")
    print("  - Pin 6   -> GND")
    print()
    
    i2c = SoftwareI2C(SCL_PIN, SDA_PIN)
    
    try:
        # Scan for devices
        print("Scanning I2C bus...")
        devices = i2c.scan()
        
        if devices:
            print(f"\n✓ Found {len(devices)} device(s):")
            for addr in devices:
                print(f"  - 0x{addr:02X}")
        else:
            print("\n✗ No I2C devices found!")
            print("\nTroubleshooting:")
            print("  1. Check wiring: VCC, GND, SDA, SCL")
            print("  2. Verify MCP23017 power LED is on")
            print("  3. Check address jumpers (A0/A1/A2)")
            i2c.cleanup()
            return False
        
        # Test MCP23017
        if MCP23017_ADDR in devices:
            print(f"\n✓ MCP23017 found at 0x{MCP23017_ADDR:02X}!")
            
            # Configure Port A as outputs
            print("\nConfiguring MCP23017 Port A as outputs...")
            if i2c.write_register(MCP23017_ADDR, IODIRA, 0x00):
                print("  ✓ IODIRA set to 0x00 (all outputs)")
            else:
                print("  ✗ Failed to write IODIRA")
                i2c.cleanup()
                return False
            
            # Test LEDs
            print("\nTesting LEDs (PA0-PA3)...")
            led_names = ["PA0 (Red?)", "PA1 (Blue?)", "PA2 (Green?)", "PA3 (Yellow?)"]
            
            for i, name in enumerate(led_names):
                print(f"\n  Turning on {name}...")
                value = 1 << i
                if i2c.write_register(MCP23017_ADDR, GPIOA, value):
                    print(f"    ✓ GPIOA = 0x{value:02X}")
                    input(f"    Press Enter when done viewing {name}...")
                else:
                    print(f"    ✗ Failed to write GPIOA")
            
            # All LEDs on
            print("\n  Turning ALL LEDs on...")
            if i2c.write_register(MCP23017_ADDR, GPIOA, 0x0F):
                print("    ✓ GPIOA = 0x0F")
                input("    Press Enter to turn off...")
            
            # All LEDs off
            i2c.write_register(MCP23017_ADDR, GPIOA, 0x00)
            print("\n✓ Test complete!")
            
        else:
            print(f"\n✗ MCP23017 not found at expected address 0x{MCP23017_ADDR:02X}")
            print(f"   Found devices at: {['0x%02X' % a for a in devices]}")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        i2c.cleanup()
    
    return True


if __name__ == "__main__":
    main()

