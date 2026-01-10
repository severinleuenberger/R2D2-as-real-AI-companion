# LED Expansion Test Scripts

**Purpose:** Test and verify MCP23017 LED system before ROS2 integration

---

## Hardware Setup Order

Follow these scripts in order during hardware installation:

### 1. Hardware Verification (Before Connecting to MCP23017)
```bash
# Read this first - use lab power supply to test LED boards
cat HARDWARE_VERIFICATION_GUIDE.md
```

### 2. Install Python Libraries (After MCP23017 wired to Jetson)
```bash
./install_mcp23017_libraries.sh
```

### 3. Verify I2C Detection
```bash
# Should show device at address 0x20
sudo i2cdetect -y 1
```

### 4. Basic LED Test (After LEDs wired to MCP23017)
```bash
# Tests all 4 LEDs with various patterns
python3 test_mcp23017_4led_basic.py
```

### 5. LED Mapping Test (Identify wire colors)
```bash
# Interactive test to determine which PA pin controls which LED color
python3 test_mcp23017_led_mapping.py
```

---

## Files

| File | Purpose | When to Use |
|------|---------|-------------|
| `HARDWARE_VERIFICATION_GUIDE.md` | Pre-wiring verification checklist | Before connecting to MCP23017 |
| `install_mcp23017_libraries.sh` | Install Adafruit libraries | After MCP23017 connected |
| `test_mcp23017_4led_basic.py` | Basic hardware test (all LEDs) | After all LEDs wired |
| `test_mcp23017_led_mapping.py` | Identify PA→color mapping | Before ROS2 integration |

---

## Expected Hardware

### MCP23017 Connections
- Jetson Pin 3 (SDA) → MCP23017 SDA header
- Jetson Pin 5 (SCL) → MCP23017 SCL header  
- Jetson Pin 1 (3.3V) → MCP23017 VCC header
- Jetson Pin 6 (GND) → MCP23017 GND header

### LED Connections
- Board #1 (3-wire): Black=GND, Red/Blue=PA0/PA1 (colors may not match!)
- Green LED: Added to Board #1, wire to PA2
- Yellow LED: 3mm LED with 220Ω resistor to PA3

---

## Troubleshooting

**"MCP23017 not detected at 0x20"**
- Check I2C wiring (4 wires: VCC, GND, SDA, SCL)
- Verify power: Check if MCP23017 power LED is on
- Try: `sudo i2cdetect -y 0` or `sudo i2cdetect -y 8`
- Check address jumpers on MCP23017 board

**"Import error: No module named 'board'"**
- Run: `./install_mcp23017_libraries.sh`
- Verify: `pip3 list | grep adafruit`

**"LEDs not lighting up"**
- Check LED polarity (longer leg to MCP23017 output, shorter to resistor/GND)
- Verify resistors are connected
- Test with multimeter: PA0-PA3 should read 3.3V when LED is "on"
- Try reversing LED (if polarity wrong, LED won't damage)

---

## Next Steps

After all tests pass:
1. Record PA→color mapping from test_mcp23017_led_mapping.py
2. Update mcp23017_status_led.launch.py with correct pin mapping
3. Build ROS2 package: `cd ~/dev/r2d2/ros2_ws && colcon build --packages-select r2d2_audio`
4. Test with ROS2: `ros2 launch r2d2_audio mcp23017_status_led.launch.py`
5. Verify with minimal_monitor.py

---

**Date:** January 9, 2026  
**Status:** Ready for hardware testing

