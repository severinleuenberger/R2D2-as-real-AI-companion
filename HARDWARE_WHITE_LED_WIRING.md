# White LED Status Indicator - Hardware Wiring Guide

**Date:** December 17, 2025  
**Platform:** NVIDIA Jetson AGX Orin 64GB  
**LED Module:** White LED Panel (16 SMD LEDs, 3V DC, 20-50mA)

---

## Overview

This guide provides detailed wiring instructions for connecting the white LED panel to the Jetson AGX Orin 40-pin GPIO header for person recognition status indication.

**Status Mapping:**
- **RED state (person recognized):** LED ON (solid)
- **BLUE state (lost/idle):** LED OFF
- **GREEN state (unknown person):** LED OFF

---

## Hardware Specifications

### White LED Panel
- **Type:** Non-addressable white LED array (16 SMD LEDs)
- **Voltage:** 3V DC
- **Current Draw:** 20-50mA total at full brightness
- **Control:** Simple on/off (all LEDs light together)
- **Connector:** 3 wires (Red, Blue, Black/Third wire)
- **Forward Voltage:** ~2.8-3.2V per LED (parallel circuit)

### GPIO Pin Assignment (DO NOT CHANGE)
- **GPIO 17** - Used for white LED control
- **Physical Pin 22** - Location on 40-pin header
- **GPIO 27** - Reserved (not used in white LED mode)
- **GPIO 22** - Reserved (not used in white LED mode)

---

## Jetson 40-Pin Header Pinout

```
              NVIDIA JETSON AGX ORIN 40-PIN HEADER
┌─────────────────────────────────────────────────────────────┐
│  Pin 1  [3.3V]    ◉ ◉ [5.0V]       Pin 2                    │
│  Pin 3  [I2C5_DAT] ◉ ◉ [5.0V]       Pin 4                    │
│  Pin 5  [I2C5_CLK] ◉ ◉ [GND]        Pin 6   ← BLACK WIRE     │
│  Pin 7  [MCLK05]   ◉ ◉ [UART1_TX]   Pin 8                    │
│  Pin 9  [GND]      ◉ ◉ [UART1_RX]   Pin 10                   │
│  Pin 11 [UART_RTS] ◉ ◉ [I2S2_CLK]   Pin 12                   │
│  Pin 13 [GPIO32]   ◉ ◉ [GND]        Pin 14                   │
│  Pin 15 [GPIO27]   ◉ ◉ [GPIO08]     Pin 16                   │
│  Pin 17 [3.3V]    ◉ ◉ [GPIO35]     Pin 18  ← RED WIRE       │
│  Pin 19 [SPI1_MOSI]◉ ◉ [GND]        Pin 20                   │
│  Pin 21 [SPI1_MISO]◉ ◉ [GPIO17]    Pin 22  ← BLUE WIRE      │
│  Pin 23 [SPI1_SCK] ◉ ◉ [SPI1_CS0_N] Pin 24                   │
│  Pin 25 [GND]      ◉ ◉ [I2C_CLK]    Pin 26                   │
│  Pin 27 [CAN0_DIN] ◉ ◉ [I2C_CLK]    Pin 28                   │
│  Pin 29 [CAN0_DOUT]◉ ◉ [GND]        Pin 30                   │
│  Pin 31 [CAN1_DOUT]◉ ◉ [GPIO09]     Pin 32                   │
│  Pin 33 [GPIO]     ◉ ◉ [GND]        Pin 34                   │
│  Pin 35 [I2S_FS]   ◉ ◉ [UART1_CTS]  Pin 36                   │
│  Pin 37 [CAN1_DIN] ◉ ◉ [I2S_SDIN]   Pin 38                   │
│  Pin 39 [GND]      ◉ ◉ [I2S_SDOUT]  Pin 40                   │
└─────────────────────────────────────────────────────────────┘

Reference: See attached pinout diagram photo
```

---

## Wiring Instructions

### Standard Configuration (Direct GPIO Connection)

**Step-by-step wiring:**

1. **Red Wire (Power/Positive):**
   - Connect to **Pin 1** (3.3V) or **Pin 17** (3.3V) on 40-pin header
   - This provides power to the LED array

2. **Blue Wire (Control Signal):**
   - Connect to **Pin 22** (GPIO 17) on 40-pin header
   - This is the control signal (HIGH = ON, LOW = OFF)

3. **Third Wire (Ground/Negative):**
   - Connect to **Pin 6** (GND) on 40-pin header
   - Alternative GND pins: 9, 14, 20, 25, 30, 34, or 39

**Wiring Diagram:**
```
┌──────────────────────────────────────────┐
│   White LED Panel (3 wires)              │
│   ┌──────────────────────┐               │
│   │   [16 SMD LEDs]      │               │
│   └──┬────────┬────────┬─┘               │
│      │        │        │                  │
│    RED      BLUE    BLACK                 │
│     │         │        │                  │
│     ↓         ↓        ↓                  │
│  Pin 1     Pin 22    Pin 6                │
│  (3.3V)   (GPIO17)   (GND)                │
│     │         │        │                  │
│  ┌──┴─────────┴────────┴───┐             │
│  │  Jetson 40-Pin Header   │             │
│  └──────────────────────────┘             │
└──────────────────────────────────────────┘
```

### Current Considerations

**GPIO Current Limitations:**
- Jetson GPIO pins: Typically ~16mA max per pin (safe limit)
- White LED panel: 20-50mA current draw
- **Risk:** Direct GPIO connection may not provide sufficient current

**Testing Procedure:**
1. Connect LED as shown above
2. Test with GPIO 17 set HIGH
3. If LED is dim or doesn't light: Current insufficient
4. Solution: Add transistor driver circuit (see below)

---

## Transistor Driver Circuit (If Needed)

If direct GPIO connection doesn't provide enough current, use a transistor to drive the LED:

### Components Required
- **Transistor:** NPN 2N2222 or similar (or P-channel MOSFET)
- **Resistor:** 1kΩ (base resistor)
- **Optional:** Current-limiting resistor for LED (if not built into LED module)

### Circuit Diagram

```
                    3.3V (Pin 1 or Pin 17)
                      │
                      │
                      ├──────────────┐
                      │              │
                   [LED Red Wire]    │
                      │              │
                   [LED Module]      │
                      │              │
                   [LED Blue Wire]   │
                      │              │
                      ↓              │
                      C              │
                    ┌─┴─┐            │
                    │ Q │ (NPN 2N2222)│
                    │   │            │
                    └─┬─┘            │
                      B              │
                      │              │
GPIO 17 (Pin 22) ─────┤              │
                      │              │
                   [1kΩ]             │
                      │              │
                   [LED Black Wire]  │
                      │              │
                      ↓              │
                    GND (Pin 6)      │
                      │              │
                      └──────────────┘

NPN Transistor Connections:
  • Base (B): GPIO 17 via 1kΩ resistor
  • Collector (C): LED negative (blue wire from LED module)
  • Emitter (E): GND (black wire from LED module)

Power:
  • LED positive (red wire): 3.3V (Pin 1 or Pin 17)
```

### Wiring with Transistor
1. **GPIO 17 (Pin 22)** → 1kΩ resistor → NPN base
2. **NPN emitter** → GND (Pin 6) + LED black wire
3. **NPN collector** → LED blue wire
4. **LED red wire** → 3.3V (Pin 1 or Pin 17)

**Operation:**
- GPIO 17 HIGH: Transistor ON → LED lights
- GPIO 17 LOW: Transistor OFF → LED off

---

## Software Configuration

### ROS 2 Launch Parameters

The white LED is controlled by the `status_led_node` with the following parameters:

```bash
ros2 launch r2d2_audio all_audio_services.launch.py \
  led_mode:=white \
  led_pin_white:=17 \
  simulate_gpio:=false
```

**Parameters:**
- `led_mode`: Set to `'white'` for white LED control (default)
- `led_pin_white`: GPIO pin number (default: 17, DO NOT CHANGE)
- `simulate_gpio`: Set to `false` for real hardware (default: false)

### Testing the LED

**Manual GPIO Test (Python):**
```python
import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

# Test: Blink LED
for i in range(5):
    GPIO.output(17, GPIO.HIGH)  # LED ON
    time.sleep(1)
    GPIO.output(17, GPIO.LOW)   # LED OFF
    time.sleep(1)

# Cleanup
GPIO.cleanup()
```

**Expected Behavior:**
- LED should blink 5 times (1 second on, 1 second off)
- If LED is very dim or doesn't light: Add transistor circuit

---

## Troubleshooting

### Issue: LED Doesn't Light

**Diagnosis:**
1. Check voltage with multimeter:
   - Pin 1 or 17 to Pin 6: Should measure ~3.3V
   - Pin 22 to Pin 6 when GPIO HIGH: Should measure ~3.3V

2. Verify wiring:
   - Red wire to 3.3V (Pin 1 or 17)
   - Blue wire to GPIO 17 (Pin 22)
   - Black wire to GND (Pin 6)

**Solutions:**
- Verify correct GPIO pin (Pin 22 = GPIO 17, not physical pin 17)
- Check if LED polarity is correct (reverse red/black if needed)
- Add transistor circuit if current is insufficient

### Issue: LED is Dim

**Cause:** GPIO current limitation (~16mA) insufficient for LED (20-50mA)

**Solution:**
- Implement transistor driver circuit (see above)
- Use external 3.3V power supply (not GPIO) with transistor control

### Issue: Wrong Status Display

**Symptom:** LED behavior doesn't match expected state

**Check:**
```bash
# Monitor status topic
ros2 topic echo /r2d2/audio/person_status

# Check LED node logs
ros2 node info /status_led_controller
```

---

## Safety Notes

1. **Never exceed GPIO voltage:** Jetson GPIO pins are 3.3V logic, NOT 5V tolerant
2. **Current limit:** Keep GPIO current draw under 16mA per pin
3. **Power cycling:** Always shutdown Jetson before disconnecting/connecting LEDs
4. **ESD protection:** Use anti-static precautions when handling components

---

## References

- **Architecture Documentation:** [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md)
- **Person Recognition Reference:** [`100_PERSON_RECOGNITION_REFERENCE.md`](100_PERSON_RECOGNITION_REFERENCE.md)
- **Jetson GPIO Documentation:** [NVIDIA Jetson GPIO Library](https://github.com/NVIDIA/jetson-gpio)
- **Pinout Diagram:** See attached 40-pin header photo

---

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Status:** Complete and tested (direct GPIO method)  
**Hardware:** White LED Panel + Jetson AGX Orin 40-pin GPIO header

