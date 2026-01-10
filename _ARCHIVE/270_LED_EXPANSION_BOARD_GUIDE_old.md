# ⚠️ ARCHIVED DOCUMENT - OUTDATED

> **This document has been superseded by the new LED implementation.**  
> **See current documentation:** [../270_LED_INSTALLATION.md](../270_LED_INSTALLATION.md)
>
> **Archived Date:** January 9, 2026  
> **Reason:** Replaced with actual hardware implementation and R2D2-specific integration
>
> This document is kept for historical reference only.

---

# LED Expansion Board Guide for R2D2 (ARCHIVED)
**Date:** December 2025  
**Purpose:** ~~Control multiple 2-wire LEDs using minimal GPIO pins~~ ARCHIVED

---

## Executive Summary

Instead of using one GPIO pin per LED, use an **I2C GPIO expander** or **shift register** to control many LEDs with just 2-3 GPIO pins.

**Best Option:** **MCP23017 I2C GPIO Expander**
- ✅ 16 LEDs per board (uses only 2 I2C pins)
- ✅ Can chain up to 8 boards (128 LEDs total)
- ✅ Simple on/off control
- ✅ ~$5-10 per board

---

## Recommended Solution: MCP23017 I2C GPIO Expander

### Hardware Overview

**MCP23017 Features:**
- **16 GPIO outputs** (8 on Port A, 8 on Port B)
- **I2C interface** (uses 2 pins: SDA, SCL)
- **3.3V or 5V logic** (Jetson compatible)
- **Can chain multiple chips** (up to 8 on one I2C bus)
- **Simple on/off control** (perfect for 2-wire LEDs)

### Jetson I2C Pin Assignment

**40-Pin GPIO Header:**
| Pin # | Physical | Signal | Function | MCP23017 Connection |
|-------|----------|--------|----------|----------------------|
| 3 | I2C5_DAT | SDA | I2C Data | SDA |
| 5 | I2C5_CLK | SCL | I2C Clock | SCL |
| 1 or 17 | 3.3V | Power | 3.3V Power | VCC |
| 6 | GND | Ground | Ground | GND |

**Note:** These I2C pins are already available and not used by your current LED setup.

### Wiring Diagram

```
Jetson 40-Pin Header:
  Pin 3 (SDA) ──────────→ MCP23017 Pin 12 (SDA)
  Pin 5 (SCL) ──────────→ MCP23017 Pin 13 (SCL)
  Pin 1/17 (3.3V) ──────→ MCP23017 Pin 9 (VCC)
  Pin 6 (GND) ──────────→ MCP23017 Pin 10 (GND)
                          MCP23017 Pin 18 (GND) ──→ Common GND

MCP23017 Outputs:
  GPA0 (Pin 21) ────────→ LED 1 (anode)
  GPA1 (Pin 22) ────────→ LED 2 (anode)
  GPA2 (Pin 23) ────────→ LED 3 (anode)
  ...
  GPA7 (Pin 28) ────────→ LED 8 (anode)
  GPB0 (Pin 1)  ────────→ LED 9 (anode)
  ...
  GPB7 (Pin 8)  ────────→ LED 16 (anode)

LED Wiring (2-wire):
  LED Anode (+) ────────→ MCP23017 output pin
  LED Cathode (-) ──────→ GND (via current-limiting resistor)
  
  OR (if LEDs have built-in resistors):
  LED (+) ──────────────→ MCP23017 output pin
  LED (-) ──────────────→ GND
```

**Current Limiting Resistors:**
- **Standard LEDs:** 220Ω-330Ω resistor between LED cathode and GND
- **LEDs with built-in resistors:** No external resistor needed
- **Current per LED:** ~10-20mA (MCP23017 can source 25mA per pin)

### Recommended Boards

**Option 1: Adafruit MCP23017 Breakout**
- **Part:** [Adafruit MCP23017 I2C GPIO Expander](https://www.adafruit.com/product/732)
- **Price:** ~$7 USD
- **Features:** Pre-soldered headers, pull-up resistors, address jumpers
- **Purchase:** Adafruit, Amazon, SparkFun

**Option 2: Generic MCP23017 Breakout**
- **Price:** ~$3-5 USD
- **Features:** Basic breakout board, may need pull-up resistors
- **Purchase:** AliExpress, eBay, Amazon

**Option 3: MCP23017 Module with Terminal Blocks**
- **Price:** ~$8-12 USD
- **Features:** Screw terminals for easy LED connections
- **Purchase:** Amazon, AliExpress

### Software Setup

**1. Enable I2C on Jetson:**
```bash
# Check if I2C is enabled
ls /dev/i2c-*

# Should show: /dev/i2c-0, /dev/i2c-1, etc.

# If not enabled, enable via:
sudo apt-get install -y i2c-tools
```

**2. Detect MCP23017:**
```bash
# Scan I2C bus 1 (default for 40-pin header)
sudo i2cdetect -y 1

# Should show device at address 0x20 (default) or 0x21-0x27 (if address jumpers set)
```

**3. Python Library:**
```bash
# Install Adafruit CircuitPython library
pip3 install adafruit-circuitpython-mcp230xx
```

**4. Example Python Code:**
```python
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MCP23017 (default address 0x20)
mcp = MCP23017(i2c)

# Set up pins as outputs
led0 = mcp.get_pin(0)  # GPA0
led1 = mcp.get_pin(1)  # GPA1
# ... up to led15 = mcp.get_pin(15)  # GPB7

# Control LEDs
led0.value = True   # Turn on LED 0
led1.value = False  # Turn off LED 1
```

---

## Alternative Solution: 74HC595 Shift Register

**If you prefer a cheaper, simpler option:**

**74HC595 Features:**
- **8 outputs per chip**
- **Uses 3 GPIO pins** (data, clock, latch)
- **Can daisy-chain** multiple chips
- **~$2-5 per chip**

**Jetson Pin Assignment:**
| Pin # | Physical | GPIO # | Function | 74HC595 Connection |
|-------|----------|--------|----------|---------------------|
| 11 | GPIO17 | GPIO 17 | Data | DS (Pin 14) |
| 13 | GPIO27 | GPIO 27 | Clock | SH_CP (Pin 11) |
| 15 | GPIO22 | GPIO 22 | Latch | ST_CP (Pin 12) |

**Note:** These pins are currently reserved for Phase 3 motors, but you could use them for LEDs if motors aren't needed yet.

**Wiring:**
```
Jetson:
  Pin 11 (GPIO17) ───→ 74HC595 Pin 14 (DS - Data)
  Pin 13 (GPIO27) ───→ 74HC595 Pin 11 (SH_CP - Clock)
  Pin 15 (GPIO22) ───→ 74HC595 Pin 12 (ST_CP - Latch)
  Pin 1/17 (3.3V) ───→ 74HC595 Pin 16 (VCC)
  Pin 6 (GND) ───────→ 74HC595 Pin 8 (GND)

74HC595 Outputs:
  Q0-Q7 (Pins 15, 1-7) → 8 LEDs (via current-limiting resistors)
```

**Python Library:**
```bash
pip3 install adafruit-circuitpython-74hc595
```

---

## Comparison Table

| Solution | LEDs per Board | GPIO Pins Used | Cost | Complexity | Best For |
|----------|----------------|----------------|------|------------|----------|
| **MCP23017** | 16 | 2 (I2C) | $5-10 | Low | **Recommended** |
| **74HC595** | 8 | 3 (SPI-like) | $2-5 | Low | Budget option |
| **Direct GPIO** | 1 | 1 | $0 | Very Low | 1-2 LEDs only |
| **PCA9685** | 16 | 2 (I2C) | $10-15 | Medium | If PWM dimming needed |

---

## Power Considerations

**Current Draw:**
- **Standard LED:** ~10-20mA per LED
- **16 LEDs @ 20mA:** 320mA total
- **MCP23017 max current:** 25mA per pin (within spec)
- **Jetson 3.3V pin max:** 50mA (insufficient for 16 LEDs)

**Power Supply Options:**

**Option 1: External 3.3V Supply (Recommended)**
```
External 3.3V Power Supply
  ├─→ MCP23017 VCC
  ├─→ LED anodes (via MCP23017 outputs)
  └─→ Common GND with Jetson
```

**Option 2: 5V Supply with Level Shifter**
```
5V Power Supply
  ├─→ MCP23017 VCC (if 5V-tolerant version)
  └─→ LEDs (5V LEDs or with appropriate resistors)
```

**Option 3: Battery Rail (14.8V) with Buck Converter**
```
14.8V Battery → 3.3V Buck Converter → MCP23017 + LEDs
```

---

## Integration with ROS 2

**Example ROS 2 Node:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017

class LedExpanderNode(Node):
    def __init__(self):
        super().__init__('led_expander_node')
        
        # Initialize MCP23017
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mcp = MCP23017(i2c)
        
        # Set up LED pins
        self.leds = [self.mcp.get_pin(i) for i in range(16)]
        for led in self.leds:
            led.switch_to_output()
        
        # Subscribe to LED control topics
        for i in range(16):
            self.create_subscription(
                Bool, f'/r2d2/leds/led_{i}',
                lambda msg, idx=i: self.led_callback(msg, idx), 10)
    
    def led_callback(self, msg, index):
        self.leds[index].value = msg.data
        self.get_logger().info(f'LED {index}: {"ON" if msg.data else "OFF"}')
```

**Usage:**
```bash
# Turn on LED 0
ros2 topic pub /r2d2/leds/led_0 std_msgs/Bool "data: true"

# Turn off LED 5
ros2 topic pub /r2d2/leds/led_5 std_msgs/Bool "data: false"
```

---

## Purchasing Links

**MCP23017 Boards:**
- **Adafruit:** https://www.adafruit.com/product/732
- **SparkFun:** https://www.sparkfun.com/products/13601
- **Amazon:** Search "MCP23017 breakout"
- **AliExpress:** Search "MCP23017 module"

**74HC595 Boards:**
- **Adafruit:** https://www.adafruit.com/product/450
- **Amazon:** Search "74HC595 shift register breakout"
- **AliExpress:** Search "74HC595 module"

---

## Next Steps

1. **Order MCP23017 board** (recommended: Adafruit for quality, AliExpress for budget)
2. **Wire I2C connections** (4 wires: SDA, SCL, 3.3V, GND)
3. **Test I2C detection:** `sudo i2cdetect -y 1`
4. **Install Python library:** `pip3 install adafruit-circuitpython-mcp230xx`
5. **Test LED control** with simple Python script
6. **Integrate with ROS 2** node for your R2D2 system

---

## Related Documentation

- **Hardware Reference:** `002_HARDWARE_REFERENCE.md` - Complete GPIO pin assignments
- **Architecture Overview:** `001_ARCHITECTURE_OVERVIEW.md` - System integration points
- **I2C Usage:** Already used for ADS1115 ADC in volume control system

---

**Last Updated:** December 2025  
**Status:** Ready for implementation

