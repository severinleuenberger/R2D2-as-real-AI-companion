# 🚀 START HERE - MCP23017 LED System Installation

**Welcome!** This folder contains everything you need to install and test your 4-LED status system.

---

## 📋 What You'll Build

**4-LED Status Display:**
- 🔴 Red LED → Person recognized (status="red")
- 🔵 Blue LED → No person (status="blue")
- 🟢 Green LED → Unknown person (status="green")
- 💛 Yellow LED → Gesture flash (500ms)

**Uses:**
- MCP23017 I2C GPIO expander (16 outputs, using 4)
- Only 2 GPIO pins (I2C bus - Pins 3, 5)
- Your pre-assembled LED boards + simple LEDs

---

## 🎯 Installation Steps (Follow in Order)

### Step 1: Hardware Verification (BEFORE Wiring!)
```bash
cat HARDWARE_VERIFICATION_GUIDE.md
```
Use lab power supply to test Board #1 and record which wire controls which color.

### Step 2: Hardware Assembly
Follow `BUILD_AND_TEST_GUIDE.md` Phase 1:
1. Desolder green LED from Board #2
2. Solder green LED to Board #1 (add 4th wire)
3. Prepare yellow LED with 220Ω resistor
4. Shutdown Jetson
5. Wire MCP23017 I2C to Jetson (4 wires: Pins 3, 5, 1, 6)
6. Connect LED boards to MCP23017 outputs
7. Power on Jetson

### Step 3: Software Installation
```bash
./test_i2c_and_install.sh
```
This checks I2C detection and installs libraries.

### Step 4: Hardware Testing
```bash
python3 test_mcp23017_4led_basic.py
```
All 4 LEDs should light up in various patterns.

### Step 5: Wire Mapping Identification
```bash
python3 test_mcp23017_led_mapping.py
```
Interactive test - tells you which PA pin controls which LED color.

### Step 6: ROS2 Integration
Follow `BUILD_AND_TEST_GUIDE.md` Phase 2-4:
1. Update launch file if wiring is reversed
2. Build ROS2 package
3. Test with ROS2 node
4. Verify with minimal_monitor.py
5. Deploy to production

---

## 📁 Files in This Folder

| File | When to Use | Takes |
|------|-------------|-------|
| **START_HERE.md** | You're reading it! | 2 min |
| **QUICK_START_CARD.md** | Quick reference during installation | 2 min |
| **BUILD_AND_TEST_GUIDE.md** | Complete step-by-step guide | Full guide |
| **HARDWARE_VERIFICATION_GUIDE.md** | Before wiring (power supply tests) | 10 min |
| **test_i2c_and_install.sh** | After wiring (I2C + libraries) | 5 min |
| **test_mcp23017_4led_basic.py** | After libraries (hardware test) | 2 min |
| **test_mcp23017_led_mapping.py** | Before ROS2 (identify wiring) | 5 min |
| **install_mcp23017_libraries.sh** | Standalone library install | 5 min |
| **README.md** | Overview of all scripts | 2 min |

---

## ⚡ Quick Reference

### I2C Wiring (Jetson → MCP23017)
```
Pin 3 → SDA
Pin 5 → SCL
Pin 1 → VCC
Pin 6 → GND
```

### LED Outputs (MCP23017 → LEDs)
```
PA0 → Board #1 wire (red or blue - test to verify!)
PA1 → Board #1 wire (blue or red - test to verify!)
PA2 → Board #1 green wire (after you add green LED)
PA3 → Yellow LED (with 220Ω resistor)
```

### Test Commands
```bash
# Check I2C
sudo i2cdetect -y 1

# Test LEDs
python3 test_mcp23017_4led_basic.py

# Identify mapping
python3 test_mcp23017_led_mapping.py

# Build ROS2
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio

# Test ROS2
ros2 launch r2d2_audio mcp23017_status_led.launch.py

# Monitor
python3 ~/dev/r2d2/tools/minimal_monitor.py
```

---

## 🎓 Key Learning Points

**LED Polarity:**
- Longer leg = Voltage side (to MCP23017)
- Shorter leg = Ground side (to resistor → GND)
- If wrong polarity: LED won't light (just reverse it!)

**Resistors:**
- Board #1 has built-in resistors ✅
- Yellow LED needs 220Ω resistor ✅
- Without resistor: LED burns out! ⚠️

**Wire Colors:**
- Board #1 wire colors might NOT match LED colors!
- Test with power supply first
- Software can handle reversed wiring

**I2C Address:**
- Default: 0x20 (address jumpers A0/A1/A2 all open)
- Can change to 0x21-0x27 if needed

---

## 📞 Need Help?

**Hardware issues:**
- Check `BUILD_AND_TEST_GUIDE.md` troubleshooting
- Review wiring diagrams in master doc
- Test with power supply (isolates wiring vs software)

**Software issues:**
- Check I2C detection: `sudo i2cdetect -y 1`
- Check libraries: `pip3 list | grep adafruit`
- Run test scripts in isolation
- Check logs: `ros2 run r2d2_audio mcp23017_status_led_node`

**Complete reference:**
- See: `~/dev/r2d2/270_LED_INSTALLATION.md`

---

## ✅ Success Criteria

When done, you should have:
- [ ] I2C shows device at 0x20
- [ ] All 4 LEDs tested and working
- [ ] ROS2 node runs without errors
- [ ] Red LED lights when you're recognized
- [ ] Blue LED lights when no one present
- [ ] Green LED lights for unknown person
- [ ] Yellow LED flashes on gestures
- [ ] LEDs match minimal_monitor.py colors EXACTLY
- [ ] System auto-starts after reboot

---

**Estimated Total Time:** 2-3 hours (including soldering green LED)

**Start with:** `BUILD_AND_TEST_GUIDE.md` 

**Good luck!** 🎉

