# RGB LED Panel Testing Suite
## Jetson AGX Orin - WS2812B Addressable LED Testing

**Test Progress:**
- ✅ Test 1.1: Power verification (40mA @ 5V, steady red glow)
- ⏳ Test 1.2: GPIO verification (ready to run)
- ⏳ Test 1.3: Data pin identification (ready to run)
- ⏳ Test 2.1: Library installation
- ⏳ Test 3.1: Color control testing

---

## Quick Start

### Step 1: Verify GPIO Pins (CRITICAL - Run First)

```bash
cd ~/dev/r2d2/tests/rgb_led
python3 verify_gpio_pins.py
```

**This will:**
- Check Jetson.GPIO library is working
- Verify Pin 15 (GPIO27) is available
- Test alternative pins (22, 32, 33)
- Recommend best pin for WS2812B data signal

**Expected Output:**
- "Pin 15 (GPIO27) is AVAILABLE - RECOMMENDED for WS2812B data signal"

---

### Step 2: Identify Data Pin

**Wiring Setup:**
1. Red wire → Jetson Pin 2 or 4 (5V)
2. Black wire → Jetson Pin 6 (GND)
3. Test wire → 330Ω resistor → Jetson Pin 15
   - **IMPORTANT: Use 330Ω resistor for safety!**

**Run Test:**
```bash
python3 test_02_identify_data_pin.py
```

**What Happens:**
- Script will toggle GPIO Pin 15 HIGH/LOW every 0.5 seconds
- Watch LED panel carefully:
  - **Data pin:** LED will change colors, turn off, or show patterns
  - **Not data pin:** LED stays steady red (no change)

**Testing Procedure:**
1. Test wire 1 (e.g., blue) → Run script → Observe
2. If no change: Disconnect, try wire 2 (e.g., white) → Run script
3. If no change: Disconnect, try wire 3 (e.g., green) → Run script

---

## Safety Checklist

- ✅ 330Ω resistor between data wire and GPIO pin
- ✅ Power from 5V pin (Pin 2 or 4), NOT 3.3V
- ✅ Common ground (Pin 6)
- ✅ Test one wire at a time
- ✅ Use Jetson.GPIO library (NOT RPi.GPIO)

---

## Troubleshooting

### "Module not found: Jetson.GPIO"
```bash
pip3 install Jetson.GPIO
```

### "Permission denied" errors
```bash
# Add user to gpio group
sudo usermod -aG gpio $USER
sudo reboot

# Or run with sudo (not recommended long-term)
sudo python3 verify_gpio_pins.py
```

### LED doesn't change during test
- Check 330Ω resistor is connected
- Verify test wire is connected to Pin 15
- Try a different test wire (blue, white, or green)
- Confirm LED shows steady red before test

---

## Next Steps

Once data pin is identified:
1. Create test_03_library_test.py (WS2812B library compatibility)
2. Create test_04_color_control.py (RED/GREEN/BLUE testing)
3. Integrate with status_led_node.py

---

## Important Notes

**Jetson vs Raspberry Pi Differences:**
- Jetson uses different GPIO numbering
- Pin 12 ≠ GPIO18 on Jetson (it's I2S2_CLK)
- Pin 15 = GPIO27 (recommended for WS2812B)
- Always use BOARD mode (physical pin numbers)
- Use Jetson.GPIO library, NOT RPi.GPIO

**WS2812B Requirements:**
- 5V power supply
- 800kHz data signal (precise timing)
- PWM-capable GPIO pin (Pin 15 is PWM-capable)
- May need level shifter if 3.3V signal doesn't work

---

**Created:** December 18, 2025  
**Platform:** Jetson AGX Orin 64GB  
**LED Panel:** WS2812B addressable RGB (24-30 LEDs)

