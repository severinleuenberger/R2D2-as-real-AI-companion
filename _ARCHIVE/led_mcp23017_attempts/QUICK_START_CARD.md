# MCP23017 LED System - Quick Start Card

**Keep this open while working!**

---

## Hardware Connection Summary

### Jetson → MCP23017 (I2C Bus - 4 wires)
```
Pin 3 (SDA) ────→ MCP23017 SDA header
Pin 5 (SCL) ────→ MCP23017 SCL header
Pin 1 (3.3V) ───→ MCP23017 VCC header
Pin 6 (GND) ────→ MCP23017 GND header
```

### MCP23017 → LEDs (Output headers)
```
Board #1 Black  → GND header
Board #1 Red    → PA0 or PA1 (test first!)
Board #1 Blue   → PA1 or PA0 (test first!)
Board #1 Green  → PA2 header

Yellow LED long leg    → PA3 header
Yellow LED short leg   → 220Ω → GND header
```

---

## LED Identification

### How to Find LED Legs
```
    Longer = Voltage (to MCP23017 PA output)
    Shorter = Ground (to resistor → GND)
```

### Resistor Requirements
- Board #1 (red/blue): ❌ NO resistor (built-in)
- Green LED: ❓ Check Board #2 (might have resistor)
- Yellow LED: ✅ NEEDS 220Ω resistor

---

## Testing Sequence (After Wiring)

### 1. I2C Detection
```bash
sudo i2cdetect -y 1
# Should show "20" at address 0x20
```

### 2. Install Libraries
```bash
cd ~/dev/r2d2/tests/led_expansion
./install_mcp23017_libraries.sh
```

### 3. Test All LEDs
```bash
python3 test_mcp23017_4led_basic.py
# All 4 LEDs should light up in sequence
```

### 4. Identify Mapping
```bash
python3 test_mcp23017_led_mapping.py
# Record which PA pin → which LED color
```

### 5. Build & Test ROS2
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
ros2 launch r2d2_audio mcp23017_status_led.launch.py
```

### 6. Verify with Monitor
```bash
python3 ~/dev/r2d2/tools/minimal_monitor.py
# LEDs should match monitor colors exactly!
```

---

## Verification Checklist

- [ ] MCP23017 detected at 0x20
- [ ] All 4 LEDs light individually
- [ ] Red LED = status "red"
- [ ] Blue LED = status "blue"
- [ ] Green LED = status "green"
- [ ] Yellow LED = gesture flash
- [ ] Colors match minimal_monitor.py
- [ ] Auto-starts after reboot

---

## Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| I2C not found | Check wires: Pin 3→SDA, Pin 5→SCL, Pin 1→VCC, Pin 6→GND |
| LED not lighting | Reverse LED (try other polarity) |
| Wrong colors | Update launch file pa0_controls/pa1_controls |
| Yellow not flashing | Check: `ros2 topic echo /r2d2/perception/gesture_event` |

---

## Important Pin Numbers (NVIDIA Jetson)

```
Pin Layout (view from above, Pin 1 = top-left):

 1 [3.3V]●  ●[5.0V]  2
 3 [SDA]  ●  ●[5.0V]  4
 5 [SCL]  ●  ●[GND]   6
 ...
```

**Use these pins:**
- Pin 1 = 3.3V (or Pin 17)
- Pin 3 = SDA
- Pin 5 = SCL
- Pin 6 = GND

---

## Software Complete - Hardware Next!

**All code is ready.** Now you need to:
1. Wire the hardware
2. Test with scripts
3. Deploy to production

**Start here:** `BUILD_AND_TEST_GUIDE.md`

---

**Master Reference:** [../270_LED_INSTALLATION.md](../270_LED_INSTALLATION.md)

