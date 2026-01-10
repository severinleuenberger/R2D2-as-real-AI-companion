# ⚠️ ARCHIVED DOCUMENT - OUTDATED

> **This document has been superseded by the new LED implementation.**  
> **See current documentation:** [../270_LED_INSTALLATION.md](../270_LED_INSTALLATION.md)
>
> **Archived Date:** January 9, 2026  
> **Reason:** Replaced with actual hardware implementation (pre-assembled LED boards + specific R2D2 status integration)
>
> This document is kept for historical reference only.

---

# R2D2 LED Expansion System - Installation Guide (ARCHIVED)
## Step-by-Step Setup for Multiple LEDs using Waveshare MCP23017 Board

**Date:** December 31, 2025  
**Status:** ~~Complete Installation Guide~~ ARCHIVED  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Estimated Time:** 90-120 minutes  
**Product:** Waveshare MCP23017 I/O Expansion Board (berrybase.ch)

---

## Overview

This guide covers installation of the Waveshare MCP23017 I/O Expansion Board to control multiple 2-wire LEDs using only 2 GPIO pins (I2C). This solution allows you to control up to 16 LEDs per board (128 LEDs with 8 boards) without consuming valuable GPIO pins on your Jetson.

**Key Benefits:**
- Uses only 2 GPIO pins (I2C SDA/SCL) for 16 LEDs
- Can chain up to 8 boards (128 LEDs total)
- PH2.0 connectors for easy wiring (no soldering required)
- 3.3V/5V compatible (works directly with Jetson)
- Simple on/off control via ROS 2 topics

---

## Prerequisites

Before starting installation, verify you have:

### Hardware Requirements

✅ **NVIDIA Jetson AGX Orin 64GB** (or compatible Jetson)
- JetPack 6.x installed
- 40-pin GPIO header accessible
- At least 2GB free storage

✅ **Waveshare MCP23017 I/O Expansion Board**
- Product: https://www.berrybase.ch/waveshare-mcp23017-io-erweiterungsplatine-16-digitale-io-ph2.0-oder-loetpads-i2c-3-3v-5v
- Features: 16 digital I/O, PH2.0 connectors, I2C interface
- Price: ~$8-12 USD

✅ **LEDs (2-wire, any quantity up to 16 per board)**
- Standard 5mm or 3mm LEDs
- Pre-wired LEDs with resistors (recommended)
- OR bare LEDs + separate resistors

✅ **Current-Limiting Resistors** (if LEDs don't have built-in resistors)
- Recommended: 220Ω (standard value)
- Alternative: 330Ω for dimmer LEDs
- Quantity: One per LED

✅ **Wiring Options:**
- **Option A:** PH2.0 jumper wires (easiest, no soldering)
- **Option B:** Dupont jumper wires + soldering to board pads
- **Option C:** Direct soldering to board pads

✅ **Optional: External Power Supply** (if using >10 LEDs at full brightness)
- 3.3V or 5V power supply (depending on LED voltage)
- Current capacity: 50mA per LED minimum
- Ground must be shared with Jetson

### Software Requirements

✅ **ROS 2 Humble** installed and working
```bash
ros2 --version
# Should show: ROS 2 Humble
```

✅ **Python 3.10** (should be pre-installed on Jetson)
```bash
python3 --version
# Should show: Python 3.10.x
```

✅ **I2C enabled** on Jetson (typically enabled by default)
```bash
ls /dev/i2c-*
# Should show: /dev/i2c-0, /dev/i2c-1, etc.
```

✅ **Git** (for package updates if needed)
```bash
git --version
```

---

## Part 1: Hardware Setup (30-45 minutes)

### Step 1.1: Physical Board Inspection

**Unpack and Verify Board Components:**

1. **Inspect the Waveshare MCP23017 board:**
   - MCP23017 chip (16-pin DIP or SOIC)
   - PH2.0 connectors (small white 2-pin connectors)
   - Solder pads (alternative connection points)
   - Address jumpers (A0, A1, A2) - small solder bridges or headers
   - I2C connection header (4 pins: VCC, GND, SDA, SCL)

2. **Check PH2.0 Connectors vs Solder Pads:**
   - **PH2.0 connectors:** Easiest option, no soldering required
   - **Solder pads:** For permanent installation or if connectors not available
   - **Waveshare board typically has:** 16 PH2.0 connectors for LEDs, 1 PH2.0 for I2C

3. **Identify Address Jumpers (A0, A1, A2):**
   - Located on back or side of board
   - Default: All open (address 0x20)
   - Used for chaining multiple boards
   - **Leave default for single board setup**

**Board Pinout Reference:**
```
Waveshare MCP23017 I/O Expansion Board
┌─────────────────────────────────────┐
│  I2C Connection (PH2.0 or pins):    │
│  [VCC] [GND] [SDA] [SCL]            │
│                                      │
│  GPIO Port A (8 outputs):           │
│  [GPA0] [GPA1] [GPA2] [GPA3]        │
│  [GPA4] [GPA5] [GPA6] [GPA7]        │
│                                      │
│  GPIO Port B (8 outputs):           │
│  [GPB0] [GPB1] [GPB2] [GPB3]        │
│  [GPB4] [GPB5] [GPB6] [GPB7]        │
│                                      │
│  Address Jumpers: [A2] [A1] [A0]    │
└─────────────────────────────────────┘
```

### Step 1.2: I2C Wiring to Jetson

**Jetson 40-Pin GPIO Header Connections:**

```
Jetson AGX Orin 40-Pin Header (Top View)
┌─────────────────────────────────────┐
│  Pin 1  [3.3V]     ◉ ◉ [5.0V]   Pin 2   │
│  Pin 3  [I2C_SDA]  ◉ ◉ [5.0V]   Pin 4   │  ← Pin 3 to MCP23017 SDA
│  Pin 5  [I2C_SCL]  ◉ ◉ [GND]    Pin 6   │  ← Pin 5 to MCP23017 SCL
│  Pin 7  [GPIO216]  ◉ ◉ [GPIO12]  Pin 8   │     Pin 6 to MCP23017 GND
│  Pin 9  [GND]      ◉ ◉ [GPIO13]  Pin 10  │
│  Pin 11 [GPIO18]   ◉ ◉ [GPIO16]  Pin 12  │
│  Pin 13 [GPIO20]   ◉ ◉ [GND]    Pin 14  │
│  Pin 15 [GPIO35]   ◉ ◉ [GPIO08]  Pin 16  │
│  Pin 17 [3.3V]     ◉ ◉ [GPIO35]  Pin 18  │  ← Pin 17 to MCP23017 VCC
│  ...                                     │
└─────────────────────────────────────────┘
```

**Connection Table:**

| Jetson Pin | Signal | Function | MCP23017 Pin | Notes |
|------------|--------|----------|--------------|-------|
| Pin 3 | I2C5_DAT | SDA (Data) | SDA | I2C data line |
| Pin 5 | I2C5_CLK | SCL (Clock) | SCL | I2C clock line |
| Pin 1 or 17 | 3.3V | Power | VCC | Choose one 3.3V pin |
| Pin 6 | GND | Ground | GND | Common ground |

**Wiring Steps:**

1. **Power off Jetson completely** (safety first)

2. **Using PH2.0 Connectors (Recommended):**
   ```
   MCP23017 I2C PH2.0 Connector:
   [VCC] ───→ Jetson Pin 17 (3.3V) - Red wire
   [GND] ───→ Jetson Pin 6 (GND)   - Black wire
   [SDA] ───→ Jetson Pin 3 (SDA)   - Yellow/White wire
   [SCL] ───→ Jetson Pin 5 (SCL)   - Blue/White wire
   ```

3. **Using Dupont Wires (Alternative):**
   - Use female-to-female Dupont wires
   - Connect directly to Jetson 40-pin header
   - Solder other end to MCP23017 pads if needed

4. **Wire Color Convention (Suggested):**
   - Red: 3.3V power
   - Black: Ground
   - Yellow/White: SDA (data)
   - Blue/Green: SCL (clock)

**⚠️ CRITICAL SAFETY CHECKS:**
- ✅ Verify 3.3V goes to VCC (NOT 5V - could work but 3.3V is safer)
- ✅ Verify GND is connected to GND (not power!)
- ✅ Verify SDA/SCL are not swapped
- ✅ Check no short circuits between pins

### Step 1.3: LED Connections

**2-Wire LED Wiring:**

**Basic LED Connection Diagram:**
```
LED with Built-in Resistor (Easiest):
┌──────────────────────────────────────┐
│  MCP23017 Output (e.g., GPA0)       │
│         │                            │
│         ├───→ LED Anode (+) [Long leg]
│         │                            │
│         └───→ LED Cathode (-) [Short leg]
│                    │                 │
│                    └──→ GND          │
└──────────────────────────────────────┘

LED without Resistor (Need Resistor):
┌──────────────────────────────────────┐
│  MCP23017 Output (e.g., GPA0)       │
│         │                            │
│         ├───→ LED Anode (+) [Long leg]
│         │                            │
│  LED Cathode (-) [Short leg]        │
│         │                            │
│     [220Ω Resistor]                  │
│         │                            │
│         └──→ GND                     │
└──────────────────────────────────────┘
```

**Connection Steps:**

1. **Identify LED Polarity:**
   - **Anode (+):** Longer leg, connects to MCP23017 output
   - **Cathode (-):** Shorter leg, connects to GND (via resistor if needed)

2. **For LEDs with Built-in Resistors:**
   ```bash
   # Most pre-wired LEDs have resistors
   LED Wire 1 (typically red/+) → MCP23017 GPA0-GPA7 or GPB0-GPB7
   LED Wire 2 (typically black/-) → Common GND rail
   ```

3. **For LEDs without Built-in Resistors:**
   ```bash
   # Add 220Ω resistor in series with cathode
   LED Anode (+) → MCP23017 output pin
   LED Cathode (-) → 220Ω resistor → GND
   ```

4. **Using PH2.0 Connectors on Waveshare Board:**
   - Waveshare board has 16 PH2.0 connectors (one per output)
   - Each connector: [Output] [GND]
   - Simply plug in LED with PH2.0 cable
   - Polarity: Check board markings (+/- or signal/GND)

**Pin Assignment Reference:**

| MCP23017 Pin | GPIO Name | LED Number | PH2.0 Connector |
|--------------|-----------|------------|-----------------|
| Pin 21 | GPA0 | LED 0 | Connector 1 |
| Pin 22 | GPA1 | LED 1 | Connector 2 |
| Pin 23 | GPA2 | LED 2 | Connector 3 |
| Pin 24 | GPA3 | LED 3 | Connector 4 |
| Pin 25 | GPA4 | LED 4 | Connector 5 |
| Pin 26 | GPA5 | LED 5 | Connector 6 |
| Pin 27 | GPA6 | LED 6 | Connector 7 |
| Pin 28 | GPA7 | LED 7 | Connector 8 |
| Pin 1 | GPB0 | LED 8 | Connector 9 |
| Pin 2 | GPB1 | LED 9 | Connector 10 |
| Pin 3 | GPB2 | LED 10 | Connector 11 |
| Pin 4 | GPB3 | LED 11 | Connector 12 |
| Pin 5 | GPB4 | LED 12 | Connector 13 |
| Pin 6 | GPB5 | LED 13 | Connector 14 |
| Pin 7 | GPB6 | LED 14 | Connector 15 |
| Pin 8 | GPB7 | LED 15 | Connector 16 |

**Current Limiting Resistor Values:**

| LED Type | Voltage Drop | Recommended Resistor | LED Brightness |
|----------|-------------|----------------------|----------------|
| Standard Red | 1.8-2.2V | 220Ω | Bright |
| Standard Green | 2.0-2.4V | 220Ω | Bright |
| Standard Blue | 3.0-3.4V | 150Ω-220Ω | Bright |
| White LED | 3.0-3.6V | 150Ω-220Ω | Bright |
| Any LED (dim) | Any | 330Ω-470Ω | Dimmer |

**Formula:** R = (Vsupply - Vled) / Iled  
**Example:** (3.3V - 2.0V) / 0.015A = 87Ω → use 220Ω for safety

### Step 1.4: Power Considerations

**Current Draw Analysis:**

- **MCP23017 max current per pin:** 25mA
- **Standard LED current:** 10-20mA
- **Total current (16 LEDs @ 20mA):** 320mA

**Power Supply Options:**

**Option A: Jetson 3.3V Pin (For ≤10 LEDs)**
```
Jetson Pin 17 (3.3V) → MCP23017 VCC
   ├─→ MCP23017 logic power (~10mA)
   └─→ LED outputs (10 LEDs × 20mA = 200mA)
Total: ~210mA (within Jetson 3.3V pin limit of ~500mA)
```

**Option B: External 3.3V Supply (For >10 LEDs or Safety)**
```
External 3.3V Power Supply (1A capacity)
   ├─→ MCP23017 VCC
   ├─→ Common GND with Jetson
   └─→ LED outputs (16 LEDs × 20mA = 320mA)
Jetson only provides I2C signals (SDA, SCL)
```

**Option C: 5V Supply with 5V-Tolerant Configuration**
```
External 5V Supply → MCP23017 VCC (if using 5V version)
   └─→ Use level shifter for I2C (3.3V ↔ 5V)
Not recommended unless necessary
```

**⚠️ Power Safety:**
- Use external power if more than 10 LEDs
- Never exceed 25mA per MCP23017 output pin
- Monitor Jetson 3.3V rail temperature if powering from it

### Step 1.5: Address Configuration (For Multiple Boards)

**Single Board Setup (Default):**
- Leave all address jumpers open (A0=0, A1=0, A2=0)
- Default I2C address: 0x20
- No configuration needed

**Multiple Board Setup:**

**I2C Address Table:**

| A2 | A1 | A0 | I2C Address | Hex | Board # |
|----|----|----|-------------|-----|---------|
| 0 | 0 | 0 | 32 | 0x20 | Board 1 |
| 0 | 0 | 1 | 33 | 0x21 | Board 2 |
| 0 | 1 | 0 | 34 | 0x22 | Board 3 |
| 0 | 1 | 1 | 35 | 0x23 | Board 4 |
| 1 | 0 | 0 | 36 | 0x24 | Board 5 |
| 1 | 0 | 1 | 37 | 0x25 | Board 6 |
| 1 | 1 | 0 | 38 | 0x26 | Board 7 |
| 1 | 1 | 1 | 39 | 0x27 | Board 8 |

**Setting Address Jumpers:**
1. Locate A0, A1, A2 pads/jumpers on board
2. Close jumper (solder bridge or jumper cap) = 1
3. Open jumper = 0
4. Set unique address for each board

**Chaining Multiple Boards:**
```
Jetson I2C Bus
   ├─→ MCP23017 Board 1 (0x20) - LEDs 0-15
   ├─→ MCP23017 Board 2 (0x21) - LEDs 16-31
   ├─→ MCP23017 Board 3 (0x22) - LEDs 32-47
   └─→ ... up to Board 8 (0x27) - LEDs 112-127

All boards share same I2C bus (SDA, SCL, VCC, GND)
```

---

## Part 2: Software Setup (20-30 minutes)

### Step 2.1: Enable I2C on Jetson

**Verify I2C is Enabled:**

```bash
# Check for I2C devices
ls /dev/i2c-*

# Expected output:
# /dev/i2c-0
# /dev/i2c-1
# /dev/i2c-2
# ... (multiple I2C buses)
```

**If I2C Not Enabled:**

```bash
# Install I2C tools
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Verify installation
i2cdetect -l

# Expected output:
# i2c-0   i2c             ...
# i2c-1   i2c             ... (This is the 40-pin header bus)
```

**Check I2C Permissions:**

```bash
# Add user to i2c group (if needed)
sudo usermod -a -G i2c $USER

# Log out and log back in for group changes to take effect
# Or use: newgrp i2c
```

### Step 2.2: Detect MCP23017 Board

**Power on Jetson and Scan I2C Bus:**

```bash
# Scan I2C bus 1 (40-pin header)
sudo i2cdetect -y 1

# Expected output (with MCP23017 at 0x20):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# ... (rest of addresses)

# If you see "20" at address 0x20, the MCP23017 is detected!
```

**Troubleshooting if Not Detected:**

1. **Check Physical Connections:**
   ```bash
   # Verify wiring:
   # - SDA to Pin 3
   # - SCL to Pin 5
   # - VCC to Pin 17 (3.3V)
   # - GND to Pin 6
   ```

2. **Try Different I2C Bus:**
   ```bash
   # Try bus 0
   sudo i2cdetect -y 0
   
   # Try bus 2
   sudo i2cdetect -y 2
   ```

3. **Check Power:**
   ```bash
   # Measure voltage at MCP23017 VCC pin
   # Should be 3.3V ±0.1V
   ```

4. **Verify Address Jumpers:**
   - Ensure A0, A1, A2 are set correctly
   - Default should be all open (0x20)

### Step 2.3: Install Python Libraries

**Install Required Libraries:**

```bash
# Update package list
sudo apt-get update

# Install Python dependencies
sudo apt-get install -y python3-pip python3-dev

# Install Adafruit Blinka (CircuitPython compatibility layer for Jetson)
pip3 install --user adafruit-blinka

# Install MCP230xx library
pip3 install --user adafruit-circuitpython-mcp230xx

# Install additional dependencies (if needed)
pip3 install --user adafruit-circuitpython-busdevice
```

**Verify Installation:**

```bash
# Test import
python3 << 'EOF'
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017
print("✅ All libraries imported successfully!")
EOF

# Expected output:
# ✅ All libraries imported successfully!
```

**Troubleshooting Installation Issues:**

```bash
# If import fails, check library versions
pip3 list | grep adafruit

# Expected output:
# adafruit-blinka           X.X.X
# adafruit-circuitpython-busdevice  X.X.X
# adafruit-circuitpython-mcp230xx   X.X.X

# If missing, reinstall
pip3 install --user --upgrade adafruit-circuitpython-mcp230xx
```

### Step 2.4: Basic LED Test Script

**Create Test Script:**

```bash
# Create test directory
mkdir -p ~/dev/r2d2/tests/led_expansion
cd ~/dev/r2d2/tests/led_expansion

# Create test script
nano test_mcp23017_basic.py
```

**Copy this test script:**

```python
#!/usr/bin/env python3
"""
Basic MCP23017 LED Test Script
Tests all 16 output pins with sequential blinking
"""

import time
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017

def main():
    print("🔧 MCP23017 LED Test Script")
    print("=" * 50)
    
    try:
        # Initialize I2C bus
        print("Initializing I2C bus...")
        i2c = busio.I2C(board.SCL, board.SDA)
        print("✅ I2C bus initialized")
        
        # Initialize MCP23017 (default address 0x20)
        print("Connecting to MCP23017 at address 0x20...")
        mcp = MCP23017(i2c, address=0x20)
        print("✅ MCP23017 connected")
        
        # Configure all 16 pins as outputs
        print("Configuring pins as outputs...")
        led_pins = []
        for i in range(16):
            pin = mcp.get_pin(i)
            pin.direction = digitalio.Direction.OUTPUT
            pin.value = False  # Start with all LEDs off
            led_pins.append(pin)
        print("✅ All 16 pins configured as outputs")
        
        print("\n" + "=" * 50)
        print("Starting LED test sequence...")
        print("Press Ctrl+C to stop")
        print("=" * 50 + "\n")
        
        # Test 1: Sequential blink (one at a time)
        print("Test 1: Sequential blink (one LED at a time)")
        for cycle in range(2):  # 2 cycles
            for i, pin in enumerate(led_pins):
                print(f"  LED {i} ON", end="\r")
                pin.value = True
                time.sleep(0.2)
                pin.value = False
        print("✅ Test 1 complete" + " " * 20)
        
        # Test 2: All on, all off
        print("\nTest 2: All LEDs on/off")
        for cycle in range(3):
            print(f"  All LEDs ON (cycle {cycle+1}/3)", end="\r")
            for pin in led_pins:
                pin.value = True
            time.sleep(0.5)
            print(f"  All LEDs OFF (cycle {cycle+1}/3)", end="\r")
            for pin in led_pins:
                pin.value = False
            time.sleep(0.5)
        print("✅ Test 2 complete" + " " * 20)
        
        # Test 3: Wave pattern
        print("\nTest 3: Wave pattern")
        for cycle in range(2):
            # Forward wave
            for i in range(16):
                if i > 0:
                    led_pins[i-1].value = False
                led_pins[i].value = True
                time.sleep(0.1)
            # Reverse wave
            for i in range(15, -1, -1):
                if i < 15:
                    led_pins[i+1].value = False
                led_pins[i].value = True
                time.sleep(0.1)
            led_pins[0].value = False
        print("✅ Test 3 complete" + " " * 20)
        
        # Turn all LEDs off
        print("\nTurning all LEDs off...")
        for pin in led_pins:
            pin.value = False
        
        print("\n" + "=" * 50)
        print("✅ All tests completed successfully!")
        print("=" * 50)
        
    except ValueError as e:
        print(f"\n❌ Error: Could not find MCP23017 at address 0x20")
        print(f"   {str(e)}")
        print("\nTroubleshooting:")
        print("1. Check I2C wiring (SDA, SCL, VCC, GND)")
        print("2. Run: sudo i2cdetect -y 1")
        print("3. Verify MCP23017 address jumpers")
        return 1
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        # Turn all LEDs off
        for pin in led_pins:
            pin.value = False
        return 0
        
    except Exception as e:
        print(f"\n❌ Unexpected error: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
```

**Make Script Executable:**

```bash
chmod +x test_mcp23017_basic.py
```

**Run Test Script:**

```bash
python3 test_mcp23017_basic.py
```

**Expected Output:**

```
🔧 MCP23017 LED Test Script
==================================================
Initializing I2C bus...
✅ I2C bus initialized
Connecting to MCP23017 at address 0x20...
✅ MCP23017 connected
Configuring pins as outputs...
✅ All 16 pins configured as outputs

==================================================
Starting LED test sequence...
Press Ctrl+C to stop
==================================================

Test 1: Sequential blink (one LED at a time)
✅ Test 1 complete

Test 2: All LEDs on/off
✅ Test 2 complete

Test 3: Wave pattern
✅ Test 3 complete

Turning all LEDs off...

==================================================
✅ All tests completed successfully!
==================================================
```

**If Test Fails:**
- Check physical LED connections
- Verify resistors are correct value
- Check LED polarity (anode to output, cathode to GND)
- Measure voltage at LED pins with multimeter

---

## Part 3: ROS 2 Integration (30-45 minutes)

### Step 3.1: Create LED Control Node

**Navigate to r2d2_audio Package:**

```bash
cd ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio
```

**Create New Node File:**

```bash
nano mcp23017_led_node.py
```

**Copy this ROS 2 node implementation:**

```python
#!/usr/bin/env python3
"""
MCP23017 LED Control Node for R2D2
Controls up to 16 LEDs via MCP23017 I2C GPIO expander

Subscribes to:
  /r2d2/leds/led_0 through /r2d2/leds/led_15 (std_msgs/Bool)
  /r2d2/leds/all (std_msgs/Bool) - Control all LEDs at once
  
Services:
  /r2d2/leds/set_pattern (std_srvs/SetBool) - Set LED pattern
  
Parameters:
  i2c_bus: I2C bus number (default: 1)
  i2c_address: MCP23017 address (default: 0x20)
  num_leds: Number of LEDs connected (default: 16)
  enabled: Enable/disable node (default: true)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time

try:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017
    HAS_MCP23017 = True
except ImportError:
    HAS_MCP23017 = False


class MCP23017LEDNode(Node):
    """ROS 2 node for controlling LEDs via MCP23017 I2C GPIO expander."""
    
    def __init__(self):
        super().__init__('mcp23017_led_node')
        
        self.get_logger().info("MCP23017 LED Node initializing...")
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x20)
        self.declare_parameter('num_leds', 16)
        self.declare_parameter('enabled', True)
        self.declare_parameter('simulate', not HAS_MCP23017)
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.num_leds = self.get_parameter('num_leds').value
        self.enabled = self.get_parameter('enabled').value
        self.simulate = self.get_parameter('simulate').value
        
        # LED state tracking
        self.led_states = [False] * 16
        self.led_pins = []
        self.mcp = None
        
        # Initialize hardware
        if not self.simulate and HAS_MCP23017:
            self._init_mcp23017()
        else:
            if self.simulate:
                self.get_logger().warn("🔶 Running in simulation mode (no hardware)")
            else:
                self.get_logger().warn("🔶 MCP23017 library not available - simulation mode")
        
        # Create subscribers for individual LEDs
        self.led_subs = []
        for i in range(self.num_leds):
            sub = self.create_subscription(
                Bool,
                f'/r2d2/leds/led_{i}',
                lambda msg, idx=i: self.led_callback(msg, idx),
                10
            )
            self.led_subs.append(sub)
        
        # Create subscriber for all LEDs
        self.all_sub = self.create_subscription(
            Bool,
            '/r2d2/leds/all',
            self.all_leds_callback,
            10
        )
        
        # Create service for pattern control
        self.pattern_srv = self.create_service(
            SetBool,
            '/r2d2/leds/set_pattern',
            self.pattern_service_callback
        )
        
        self.get_logger().info(f"✅ MCP23017 LED Node ready (address: 0x{self.i2c_address:02x}, LEDs: {self.num_leds})")
        self.get_logger().info(f"   Subscribed to /r2d2/leds/led_0 through /r2d2/leds/led_{self.num_leds-1}")
    
    def _init_mcp23017(self):
        """Initialize MCP23017 hardware."""
        try:
            # Initialize I2C bus
            self.get_logger().info(f"Initializing I2C bus {self.i2c_bus}...")
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize MCP23017
            self.get_logger().info(f"Connecting to MCP23017 at 0x{self.i2c_address:02x}...")
            self.mcp = MCP23017(i2c, address=self.i2c_address)
            
            # Configure pins as outputs
            self.get_logger().info("Configuring pins as outputs...")
            for i in range(self.num_leds):
                pin = self.mcp.get_pin(i)
                pin.direction = digitalio.Direction.OUTPUT
                pin.value = False  # Start with LEDs off
                self.led_pins.append(pin)
            
            self.get_logger().info("✅ MCP23017 initialized successfully")
            
        except ValueError as e:
            self.get_logger().error(f"❌ Could not find MCP23017 at 0x{self.i2c_address:02x}: {str(e)}")
            self.get_logger().error("   Falling back to simulation mode")
            self.simulate = True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error initializing MCP23017: {str(e)}")
            self.get_logger().error("   Falling back to simulation mode")
            self.simulate = True
    
    def led_callback(self, msg, led_index):
        """Handle individual LED control messages."""
        if not self.enabled:
            return
        
        state = msg.data
        self.led_states[led_index] = state
        
        if not self.simulate and self.led_pins:
            try:
                self.led_pins[led_index].value = state
                self.get_logger().debug(f"LED {led_index}: {'ON' if state else 'OFF'}")
            except Exception as e:
                self.get_logger().error(f"Error setting LED {led_index}: {str(e)}")
        else:
            self.get_logger().info(f"[SIMULATED] LED {led_index}: {'ON' if state else 'OFF'}")
    
    def all_leds_callback(self, msg):
        """Handle all LEDs control message."""
        if not self.enabled:
            return
        
        state = msg.data
        self.get_logger().info(f"Setting all LEDs: {'ON' if state else 'OFF'}")
        
        for i in range(self.num_leds):
            self.led_states[i] = state
            
            if not self.simulate and self.led_pins:
                try:
                    self.led_pins[i].value = state
                except Exception as e:
                    self.get_logger().error(f"Error setting LED {i}: {str(e)}")
    
    def pattern_service_callback(self, request, response):
        """Handle pattern service requests."""
        # Example: Toggle all LEDs in pattern
        if request.data:
            self.get_logger().info("Starting LED pattern...")
            # Implement your pattern here
            response.success = True
            response.message = "Pattern started"
        else:
            self.get_logger().info("Stopping LED pattern...")
            # Turn all LEDs off
            for i in range(self.num_leds):
                if not self.simulate and self.led_pins:
                    self.led_pins[i].value = False
            response.success = True
            response.message = "Pattern stopped"
        
        return response
    
    def destroy_node(self):
        """Clean up on node shutdown."""
        self.get_logger().info("Shutting down LED node...")
        
        # Turn all LEDs off
        if not self.simulate and self.led_pins:
            for pin in self.led_pins:
                try:
                    pin.value = False
                except Exception:
                    pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MCP23017LEDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Make Node Executable:**

```bash
chmod +x mcp23017_led_node.py
```

**Update Package setup.py:**

```bash
cd ~/dev/r2d2/ros2_ws/src/r2d2_audio
nano setup.py
```

**Add entry point (in `console_scripts` section):**

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'mcp23017_led_node = r2d2_audio.mcp23017_led_node:main',
    ],
},
```

### Step 3.2: Build and Install Node

```bash
# Navigate to workspace root
cd ~/dev/r2d2/ros2_ws

# Build package
colcon build --packages-select r2d2_audio

# Source workspace
source install/setup.bash
```

### Step 3.3: Test ROS 2 Node

**Terminal 1 - Start Node:**

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

# Run node
ros2 run r2d2_audio mcp23017_led_node

# Expected output:
# [INFO] [mcp23017_led_node]: MCP23017 LED Node initializing...
# [INFO] [mcp23017_led_node]: Initializing I2C bus 1...
# [INFO] [mcp23017_led_node]: Connecting to MCP23017 at 0x20...
# [INFO] [mcp23017_led_node]: Configuring pins as outputs...
# [INFO] [mcp23017_led_node]: ✅ MCP23017 initialized successfully
# [INFO] [mcp23017_led_node]: ✅ MCP23017 LED Node ready (address: 0x20, LEDs: 16)
```

**Terminal 2 - Test LED Control:**

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

# Turn on LED 0
ros2 topic pub --once /r2d2/leds/led_0 std_msgs/Bool "data: true"

# Turn off LED 0
ros2 topic pub --once /r2d2/leds/led_0 std_msgs/Bool "data: false"

# Turn on LED 5
ros2 topic pub --once /r2d2/leds/led_5 std_msgs/Bool "data: true"

# Turn all LEDs on
ros2 topic pub --once /r2d2/leds/all std_msgs/Bool "data: true"

# Turn all LEDs off
ros2 topic pub --once /r2d2/leds/all std_msgs/Bool "data: false"
```

### Step 3.4: Create Launch File

**Create launch file:**

```bash
cd ~/dev/r2d2/ros2_ws/src/r2d2_audio/launch
nano mcp23017_led.launch.py
```

**Copy this launch configuration:**

```python
#!/usr/bin/env python3
"""
Launch file for MCP23017 LED control node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (typically 1 for 40-pin header)'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x20',
        description='MCP23017 I2C address (0x20-0x27)'
    )
    
    num_leds_arg = DeclareLaunchArgument(
        'num_leds',
        default_value='16',
        description='Number of LEDs connected (1-16)'
    )
    
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable LED control node'
    )
    
    # Create node
    mcp23017_led_node = Node(
        package='r2d2_audio',
        executable='mcp23017_led_node',
        name='mcp23017_led_node',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': int(LaunchConfiguration('i2c_address'), 16),
            'num_leds': LaunchConfiguration('num_leds'),
            'enabled': LaunchConfiguration('enabled'),
        }]
    )
    
    return LaunchDescription([
        i2c_bus_arg,
        i2c_address_arg,
        num_leds_arg,
        enabled_arg,
        mcp23017_led_node,
    ])
```

**Test Launch File:**

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash

# Launch node
ros2 launch r2d2_audio mcp23017_led.launch.py

# Launch with custom parameters
ros2 launch r2d2_audio mcp23017_led.launch.py i2c_address:=0x21 num_leds:=8
```

### Step 3.5: Integration with Existing Status LED

**Option A: Replace Existing GPIO LED with MCP23017**

Edit `status_led_node.py` to publish to `/r2d2/leds/led_0` instead of controlling GPIO directly:

```python
# In status_led_node.py, replace GPIO control with topic publishing:
self.led_pub = self.create_publisher(Bool, '/r2d2/leds/led_0', 10)

# In _update_led method:
def _update_led(self, status):
    msg = Bool()
    msg.data = (status == "red")  # LED ON for red status
    self.led_pub.publish(msg)
```

**Option B: Run Both (GPIO + MCP23017)**

Keep existing GPIO LED and add MCP23017 LEDs for additional indicators:
- GPIO LED: Main status indicator (existing)
- MCP23017 LED 0: Duplicate status
- MCP23017 LEDs 1-15: Additional indicators (future use)

---

## Part 4: Testing & Verification (15 minutes)

### Step 4.1: Hardware Verification Checklist

✅ **Physical Connections:**
```bash
# Visual inspection:
# [ ] I2C wires connected correctly (Pin 3, 5, 6, 17)
# [ ] LED polarity correct (anode to output, cathode to GND)
# [ ] Resistors in place (if needed)
# [ ] No short circuits
# [ ] Power LED on MCP23017 lit (if present)
```

✅ **I2C Detection:**
```bash
# Verify MCP23017 detected
sudo i2cdetect -y 1

# Should show device at 0x20 (or configured address)
```

✅ **LED Physical Test:**
```bash
# Run basic test script
python3 ~/dev/r2d2/tests/led_expansion/test_mcp23017_basic.py

# All LEDs should blink in sequence
```

### Step 4.2: ROS 2 Node Testing

**Start Node:**
```bash
ros2 run r2d2_audio mcp23017_led_node
```

**Test Individual LED Control:**
```bash
# Open new terminal
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Test each LED
for i in {0..15}; do
  echo "Testing LED $i"
  ros2 topic pub --once /r2d2/leds/led_$i std_msgs/Bool "data: true"
  sleep 0.5
  ros2 topic pub --once /r2d2/leds/led_$i std_msgs/Bool "data: false"
  sleep 0.2
done
```

**Test All LEDs:**
```bash
# All on
ros2 topic pub --once /r2d2/leds/all std_msgs/Bool "data: true"
sleep 2

# All off
ros2 topic pub --once /r2d2/leds/all std_msgs/Bool "data: false"
```

**Check Node Status:**
```bash
# List topics
ros2 topic list | grep leds

# Expected:
# /r2d2/leds/all
# /r2d2/leds/led_0
# /r2d2/leds/led_1
# ...
# /r2d2/leds/led_15

# Check node info
ros2 node info /mcp23017_led_node
```

### Step 4.3: Integration Testing

**Test with Person Recognition System:**

If you want to use LED 0 as status indicator:

```bash
# Terminal 1: Start LED node
ros2 run r2d2_audio mcp23017_led_node

# Terminal 2: Simulate recognition status
ros2 topic pub /r2d2/leds/led_0 std_msgs/Bool "data: true"  # Recognized
# LED 0 should turn ON

ros2 topic pub /r2d2/leds/led_0 std_msgs/Bool "data: false"  # Lost
# LED 0 should turn OFF
```

**Create LED Pattern Test:**

```bash
# Create pattern test script
cat > ~/dev/r2d2/tests/led_expansion/test_led_pattern.sh << 'EOF'
#!/bin/bash
# LED Pattern Test

source ~/dev/r2d2/ros2_ws/install/setup.bash

echo "Running LED pattern test..."

# Wave pattern
for i in {0..15}; do
  ros2 topic pub --once /r2d2/leds/led_$i std_msgs/Bool "data: true"
  sleep 0.1
done

for i in {15..0}; do
  ros2 topic pub --once /r2d2/leds/led_$i std_msgs/Bool "data: false"
  sleep 0.1
done

echo "Pattern complete"
EOF

chmod +x ~/dev/r2d2/tests/led_expansion/test_led_pattern.sh

# Run pattern
~/dev/r2d2/tests/led_expansion/test_led_pattern.sh
```

---

## Part 5: Troubleshooting

### Issue 1: I2C Device Not Detected

**Symptom:**
```bash
sudo i2cdetect -y 1
# Shows no device at 0x20
```

**Solutions:**

1. **Check Physical Wiring:**
   ```bash
   # Verify connections:
   # Jetson Pin 3 → MCP23017 SDA
   # Jetson Pin 5 → MCP23017 SCL  
   # Jetson Pin 17 → MCP23017 VCC (3.3V)
   # Jetson Pin 6 → MCP23017 GND
   ```

2. **Check Power:**
   ```bash
   # Measure voltage at MCP23017 VCC
   # Should be 3.3V ±0.1V
   # If 0V, check 3.3V connection
   ```

3. **Try Different I2C Bus:**
   ```bash
   # Try bus 0
   sudo i2cdetect -y 0
   
   # Try bus 2
   sudo i2cdetect -y 2
   ```

4. **Check Address Jumpers:**
   - Verify A0, A1, A2 are configured correctly
   - Default: all open = 0x20
   - Try scanning all addresses: `sudo i2cdetect -y 1`

5. **Check for I2C Conflicts:**
   ```bash
   # List all I2C devices
   sudo i2cdetect -y 1
   
   # If other devices at 0x20, change MCP23017 address
   ```

### Issue 2: LEDs Not Responding

**Symptom:** MCP23017 detected, but LEDs don't light up

**Solutions:**

1. **Check LED Polarity:**
   - Anode (long leg, +) → MCP23017 output
   - Cathode (short leg, -) → GND (via resistor)
   - Try reversing LED if not lighting

2. **Check Resistor Value:**
   ```bash
   # Too high resistance = dim/no light
   # Recommended: 220Ω
   # Try lower value: 150Ω (brighter)
   ```

3. **Test with Multimeter:**
   ```bash
   # Measure voltage at LED:
   # When ON: Should be ~3.3V at anode
   # When OFF: Should be ~0V at anode
   ```

4. **Test Single LED Directly:**
   ```python
   # Quick test script
   python3 << 'EOF'
   import board, busio, digitalio
   from adafruit_mcp230xx.mcp23017 import MCP23017
   i2c = busio.I2C(board.SCL, board.SDA)
   mcp = MCP23017(i2c)
   pin0 = mcp.get_pin(0)
   pin0.direction = digitalio.Direction.OUTPUT
   pin0.value = True  # Should turn LED ON
   input("Press Enter to turn OFF...")
   pin0.value = False
   EOF
   ```

### Issue 3: Power Issues

**Symptom:** Some LEDs work, others don't. Flickering.

**Solutions:**

1. **Insufficient Current:**
   ```bash
   # Jetson 3.3V pin limited to ~500mA
   # 16 LEDs × 20mA = 320mA
   # If more than 10 LEDs at full brightness, use external power
   ```

2. **Use External Power Supply:**
   ```
   External 3.3V Supply (1A) → MCP23017 VCC
   Common GND with Jetson
   Jetson provides only I2C signals
   ```

3. **Reduce LED Current:**
   ```bash
   # Use higher resistor values
   # 220Ω → 330Ω (reduces current from 15mA to 10mA)
   ```

### Issue 4: ROS 2 Node Not Starting

**Symptom:**
```bash
ros2 run r2d2_audio mcp23017_led_node
# Returns error or not found
```

**Solutions:**

1. **Rebuild Package:**
   ```bash
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_audio --cmake-clean-cache
   source install/setup.bash
   ```

2. **Check Entry Point:**
   ```bash
   # Verify setup.py has entry point
   cat ~/dev/r2d2/ros2_ws/src/r2d2_audio/setup.py | grep mcp23017
   
   # Should show:
   # 'mcp23017_led_node = r2d2_audio.mcp23017_led_node:main',
   ```

3. **Check File Permissions:**
   ```bash
   chmod +x ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/mcp23017_led_node.py
   ```

4. **Test Node Directly:**
   ```bash
   python3 ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/mcp23017_led_node.py
   ```

### Issue 5: Address Conflicts (Multiple Boards)

**Symptom:** Only one board works when multiple connected

**Solutions:**

1. **Set Unique Addresses:**
   ```bash
   # Board 1: A0=0, A1=0, A2=0 → 0x20
   # Board 2: A0=1, A1=0, A2=0 → 0x21
   # Board 3: A0=0, A1=1, A2=0 → 0x22
   ```

2. **Verify All Addresses:**
   ```bash
   sudo i2cdetect -y 1
   # Should show multiple addresses: 20, 21, 22, etc.
   ```

3. **Run Multiple Node Instances:**
   ```bash
   # Node for board 1 (0x20)
   ros2 run r2d2_audio mcp23017_led_node --ros-args -p i2c_address:=0x20
   
   # Node for board 2 (0x21)  
   ros2 run r2d2_audio mcp23017_led_node --ros-args -p i2c_address:=0x21
   ```

---

## Part 6: Advanced Configuration

### Chaining Multiple MCP23017 Boards

**Hardware Setup:**

```
Jetson I2C Bus (Pin 3, 5)
   ├─→ Board 1 (0x20) - LEDs 0-15
   ├─→ Board 2 (0x21) - LEDs 16-31
   └─→ Board 3 (0x22) - LEDs 32-47

All boards share: VCC, GND, SDA, SCL
```

**Software Setup:**

1. **Create Multi-Board Node:**

```python
# In mcp23017_led_node.py, add support for multiple boards:
self.declare_parameter('num_boards', 1)
self.num_boards = self.get_parameter('num_boards').value

# Initialize multiple MCP23017 instances:
self.mcps = []
for board_idx in range(self.num_boards):
    address = self.i2c_address + board_idx
    mcp = MCP23017(i2c, address=address)
    self.mcps.append(mcp)
```

2. **Launch Multiple Boards:**

```bash
ros2 launch r2d2_audio mcp23017_led.launch.py \
  num_boards:=3 \
  i2c_address:=0x20
```

### LED Patterns and Animations

**Create Pattern Node:**

```bash
cd ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio
nano led_pattern_node.py
```

```python
#!/usr/bin/env python3
"""LED Pattern Generator Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import threading

class LEDPatternNode(Node):
    def __init__(self):
        super().__init__('led_pattern_node')
        
        # Create publishers for all LEDs
        self.led_pubs = []
        for i in range(16):
            pub = self.create_publisher(Bool, f'/r2d2/leds/led_{i}', 10)
            self.led_pubs.append(pub)
        
        # Start pattern thread
        self.pattern_thread = threading.Thread(target=self.run_pattern, daemon=True)
        self.pattern_thread.start()
    
    def run_pattern(self):
        """Run LED pattern loop"""
        while rclpy.ok():
            # Wave pattern
            for i in range(16):
                msg = Bool()
                msg.data = True
                self.led_pubs[i].publish(msg)
                time.sleep(0.1)
                msg.data = False
                self.led_pubs[i].publish(msg)

def main():
    rclpy.init()
    node = LEDPatternNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration with Person Recognition

**Automatically control LEDs based on person status:**

```python
# In status_led_node.py or create new node:
def status_callback(self, msg):
    """Handle person status updates"""
    status_data = json.loads(msg.data)
    status = status_data.get("status", "blue")
    
    # Map status to LED index
    led_mapping = {
        "red": 0,    # Recognized - LED 0 ON
        "green": 1,  # Unknown - LED 1 ON
        "blue": 2,   # No person - LED 2 ON
    }
    
    # Turn off all status LEDs
    for i in range(3):
        msg = Bool()
        msg.data = False
        self.led_pubs[i].publish(msg)
    
    # Turn on appropriate LED
    if status in led_mapping:
        msg = Bool()
        msg.data = True
        self.led_pubs[led_mapping[status]].publish(msg)
```

---

## Part 7: Production Deployment

### Create Systemd Service (Optional)

**Create service file:**

```bash
sudo nano /etc/systemd/system/r2d2-mcp23017-led.service
```

**Service configuration:**

```ini
[Unit]
Description=R2D2 MCP23017 LED Control Service
After=network.target

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/ros2_ws
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/severin/dev/r2d2/ros2_ws/install/setup.bash && ros2 run r2d2_audio mcp23017_led_node"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable and start service:**

```bash
sudo systemctl daemon-reload
sudo systemctl enable r2d2-mcp23017-led.service
sudo systemctl start r2d2-mcp23017-led.service

# Check status
sudo systemctl status r2d2-mcp23017-led.service
```

---

## References & Documentation

### Product Links

- **Waveshare MCP23017 Board:** https://www.berrybase.ch/waveshare-mcp23017-io-erweiterungsplatine-16-digitale-io-ph2.0-oder-loetpads-i2c-3-3v-5v
- **Waveshare Documentation:** https://www.waveshare.com/wiki/MCP23017_IO_Expansion_Board
- **MCP23017 Datasheet:** https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf

### R2D2 Documentation

- **Hardware Reference:** [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) - GPIO pinout
- **Architecture Overview:** [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - System integration
- **LED Expansion Guide:** [`270_LED_EXPANSION_BOARD_GUIDE.md`](270_LED_EXPANSION_BOARD_GUIDE.md) - Board comparison

### Adafruit Libraries

- **CircuitPython MCP230xx:** https://docs.circuitpython.org/projects/mcp230xx/en/latest/
- **Adafruit Blinka:** https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

---

## Summary

You have successfully installed and configured the Waveshare MCP23017 I/O Expansion Board for LED control on your R2D2 system.

**What You Accomplished:**
- ✅ Connected MCP23017 board via I2C (2 GPIO pins)
- ✅ Wired up to 16 LEDs with proper current limiting
- ✅ Installed Python libraries (adafruit-circuitpython-mcp230xx)
- ✅ Created ROS 2 node for LED control
- ✅ Tested individual and batch LED control
- ✅ Integrated with existing R2D2 system

**Next Steps:**
1. Add more LEDs (up to 16 per board)
2. Chain multiple boards for 128 total LEDs
3. Create LED patterns for status indication
4. Integrate with person recognition system
5. Add LED animations and effects

**Quick Reference Commands:**

```bash
# Check I2C
sudo i2cdetect -y 1

# Run LED node
ros2 run r2d2_audio mcp23017_led_node

# Control LED
ros2 topic pub --once /r2d2/leds/led_0 std_msgs/Bool "data: true"

# Test pattern
python3 ~/dev/r2d2/tests/led_expansion/test_mcp23017_basic.py
```

---

**Document Status:** ✅ Complete Installation Guide  
**Last Updated:** December 31, 2025  
**Maintainer:** Severin Leuenberger

---

**End of LED Installation Guide**


