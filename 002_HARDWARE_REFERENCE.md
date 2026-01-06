# R2D2 Hardware Reference
**Date:** December 20, 2025  
**Project:** R2D2 as a Real AI Companion  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Document Version:** 1.0

---

## Executive Summary

This document provides a comprehensive hardware reference for the R2D2 project, cataloging all physical components, connections, and specifications. The system is built around the NVIDIA Jetson AGX Orin 64GB compute platform with modular subsystems for vision, audio, control, and future locomotion.

**Total Component Cost:**
- Compute + Sensors: ~$2,200
- Complete System (with DeAgostini chassis): ~$3,600

**Current Status:** Phase 1-2 Complete (Perception + Speech)  
**Integration Level:** 11 components operational, 6 components pending Phase 3-4

---

## 1. Complete Component Connection List

### 1.1 Currently Connected & Operational (Phase 1-2)

| # | Component | Model/Type | Connection Point | Pin/Port Details | Wire/Cable | Status | Phase |
|---|-----------|------------|------------------|------------------|------------|--------|-------|
| 1 | **Main Compute** | NVIDIA Jetson AGX Orin 64GB | — | Power input (7-20V DC) | Direct from battery via 10A fuse + barrel jack (5.5mm×2.5mm) | ✅ Operational | 1 |
| 2 | **Camera** | Luxonis OAK-D Lite Auto Focus | USB-C Port (USB 3.0) | Direct to Jetson USB-C port (not through hub) | Standard USB-C cable | ✅ Operational | 1 |
| 3 | **Microphone** | HyperX QuadCast S USB | USB | USB Type-A port | USB cable | ✅ Operational | 2 |
| 4 | **Speaker Amplifier** | PAM8403 3W Stereo | J511 Audio Header | Pin 9 (HPO_L - I2S Left Channel) | 2-wire audio cable | ✅ Operational | 1 |
| 5 | **Speaker** | 8Ω Speaker | PAM8403 Output | Screw terminals on amplifier | 2-wire speaker cable | ✅ Operational | 1 |
| 6 | **Status LED** | White LED Panel (16 SMD LEDs) | 40-pin GPIO Header | Pin 22 (GPIO17), Pin 1/17 (3.3V), Pin 6 (GND) | 3-wire: Red (3.3V), Blue (GPIO17), Black (GND) | ✅ Operational | 1 |
| 7 | **Shutdown Button** | Momentary Push Button | 40-pin GPIO Header | Pin 32 (GPIO32) + GND | 2-wire button cable | ✅ Operational | 1 |
| 8 | **Boot/Wake Button** | Momentary Push Button | J42 Automation Header | Pin 4 (POWER) + Pin 1 (GND) | 2-wire button cable | ⏳ Ready, not tested | 1 |
| 9 | **Main Battery** | Turnigy 4S LiPo (3× batteries) | Power Distribution Board | XT60 connector | XT60 power cable + parallel harness | ✅ Charged & Ready | 1 |
| 10 | **Power Connection (Direct)** | 10A Fuse + Barrel Jack | Between battery and Jetson | Input: XT60 from battery, Output: 5.5mm×2.5mm barrel jack (center +) | 18 AWG wire + inline fuse | ✅ Simple & Efficient | 1 |
| 11 | **Chassis** | DeAgostini R2-D2 1:2 Kit | — | Physical mounting | — | ✅ Complete | 1 |

### 1.2 Future Connections (Phase 3 - Motors & Navigation)

| # | Component | Model/Type | **Proposed Connection** | Pin/Port Details (Suggested) | Wire/Cable | Status | Phase |
|---|-----------|------------|-------------------------|------------------------------|------------|--------|-------|
| 12 | **Left Wheel Motor** | DeAgostini DC Motor w/ Encoder | Pololu G2 24v21 Driver #1 | PWM: Pin 33 (GPIO13), DIR: Pin 35 (GPIO19) | 4-wire: 2× motor power, 2× encoder | ⏳ Assembled, not wired | 3 |
| 13 | **Right Wheel Motor** | DeAgostini DC Motor w/ Encoder | Pololu G2 24v21 Driver #2 | PWM: Pin 32 (GPIO12)*, DIR: Pin 36 (GPIO16) | 4-wire: 2× motor power, 2× encoder | ⏳ Assembled, not wired | 3 |
| 14 | **Dome Motor (Head)** | DeAgostini DC Motor w/ Encoder | Pololu G2 24v21 Driver #3 (optional) | **Suggested:** PWM: Pin 15 (GPIO27), DIR: Pin 29 (GPIO5) | 4-wire: 2× motor power, 2× encoder | ⏳ Not integrated | 3 |
| 15 | **Motor Driver #1** | Pololu G2 24v21 (Left Wheel) | 40-pin GPIO + Battery | Control: GPIO PWM/DIR, Power: 14.8V LiPo direct | Multi-wire harness | ✅ Assembled, not wired | 3 |
| 16 | **Motor Driver #2** | Pololu G2 24v21 (Right Wheel) | 40-pin GPIO + Battery | Control: GPIO PWM/DIR, Power: 14.8V LiPo direct | Multi-wire harness | ✅ Assembled, not wired | 3 |
| 16a | **Motor Driver #3 (Optional)** | Pololu G2 24v21 (Dome) | 40-pin GPIO + Battery | Control: GPIO PWM/DIR, Power: 14.8V LiPo direct | Multi-wire harness | ⏳ Optional | 3 |
| 17 | **DC-DC Converter (5V)** | Buck Converter 14.8V→5V | Between battery and servos | Input: Battery rail, Output: Servo power rail | Power cables | ⏳ Partial wiring | 3 |

**Note on Pin 32 conflict (*):** Pin 32 is currently used for shutdown button (GPIO32). Right wheel motor PWM would need alternative pin assignment (e.g., Pin 11/GPIO17 if LED moved to software control, or use Pin 13/GPIO27 if available).

### 1.3 Camera Orientation (No Additional Servos Needed)

**Note:** The DeAgostini R2-D2 dome motor provides camera rotation (pan) functionality. The OAK-D camera is mounted inside the dome, so when the dome rotates, the camera rotates with it. No separate pan/tilt servos are needed.

### 1.4 Future Connections (Phase 4 - Status & Expression)

| # | Component | Model/Type | **Proposed Connection** | Pin/Port Details (Suggested) | Wire/Cable | Status | Phase |
|---|-----------|------------|-------------------------|------------------------------|------------|--------|-------|
| 20 | **RGB LED Strip** | WS2812B Addressable (~24-30 LEDs) | 40-pin GPIO Header | **Suggested:** Pin 12 (GPIO18) - Data pin (PWM/SPI), 5V rail, GND | 3-wire: Data, 5V, GND | ⏳ Reserved | 4 |

**Design Notes (RGB LED):**
- **Interface:** WS2812B serial protocol (requires precise timing, hardware PWM/SPI)
- **Power:** External 5V rail (LEDs draw ~50-60mA each at full white)
- **Control Library:** rpi_ws281x or similar for Jetson compatibility

---

## 2. Detailed Connection Reference by Interface

### 2.1 USB Connections

#### OAK-D Lite Camera
- **Model:** Luxonis OAK-D Lite Auto Focus
- **Serial:** 19443010E1D30C7E00
- **Interface:** USB 3.0 (USB-C connector)
- **Connection:** Direct to Jetson USB port (NOT through USB hub - critical for bandwidth)
- **Power:** Bus-powered, 500mA @ 5V
- **Capabilities:**
  - RGB: 1920×1080 @ 30 FPS (actual: 1280×1080 + padding)
  - Depth: Stereo depth via OV9782 pair
  - IMU: Onboard inertial measurement unit
- **Status:** ✅ Operational
- **Phase:** 1 (Perception)

#### HyperX QuadCast S USB Microphone
- **Model:** HyperX QuadCast S
- **Interface:** USB Type-A
- **Connection:** Direct to Jetson USB port
- **Specifications:**
  - Native: 48kHz stereo capture
  - Resampled: 24kHz mono (for OpenAI Realtime API)
  - Auto-detection: By device name matching ("hyperx" or "quadcast")
- **Status:** ✅ Operational
- **Phase:** 2 (Speech)

---

### 2.2 Jetson Power Connection (DC Barrel Jack - DIRECT from Battery!)

#### ✅ **RECOMMENDED: Direct Connection (No Converter Needed!)**

**Jetson AGX Orin DC Power Jack Specifications (Official NVIDIA):**
- **Connector:** DC barrel jack on rear panel
- **Barrel Size:** 5.5mm outer diameter (OD) × 2.5mm inner diameter (ID)
- **Polarity:** Center positive (+)
- **Voltage Range:** **7-20V DC** (official spec from NVIDIA SP-10900-001_v1.2)
- **Maximum Current:** 5.0A (from official specification Table 6-2)
- **Recommended Input:** 14.8V from 4S LiPo ✅ PERFECT!

**Connection Strategy for Turnigy 4S LiPo (14.8V):**

Your **14.8V battery voltage is already within the 7-20V specification!** No boost or buck converter needed:

```
3× Turnigy 4S LiPo (14.8V, XT60 connector, parallel)
    ↓ (18 AWG wire)
10A Inline Fuse (safety protection)
    ↓ (18 AWG wire)
5.5mm × 2.5mm Barrel Plug (center positive)
    ↓
Jetson AGX Orin DC Power Jack
```

**Why Direct Connection is Better:**
✅ **Simpler:** No DC-DC converter (fewer components, less cost)
✅ **More Efficient:** 100% efficiency (no conversion losses)
✅ **More Reliable:** Fewer failure points
✅ **Cheaper:** ~$5 (fuse + barrel jack) vs $25-50 (converter)
✅ **Within Spec:** 14.0-16.8V is perfectly within 7-20V range

**Battery Voltage Verification:**
- **Fully Charged:** 16.8V ✅ (within 7-20V range)
- **Nominal:** 14.8V ✅ (within 7-20V range)
- **Cutoff:** 14.0V ✅ (within 7-20V range)

**Required Components:**
1. **Inline Fuse Holder:** 10A automotive blade fuse holder
2. **Fuse:** 10A slow-blow (protects Jetson from short circuit)
3. **Barrel Jack Plug:** 5.5mm OD × 2.5mm ID, center-positive
4. **Wire:** 18 AWG (red + black, handles 10A safely)
5. **LiPo Alarm:** Voltage monitor (set cutoff at 14.0V to protect battery)

**Jetson Power Connector Location:**

![Jetson AGX Orin Developer Kit - DC Power Jack](https://developer.download.nvidia.com/embedded/images/jetsonAgxOrin/getting_started/jaodk_devkit_side_buttons_dcin.png)

*Image source: [NVIDIA Jetson AGX Orin Developer Kit User Guide](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html)*

**Power Jack Details (J41):**
- Mark **5** in the image above
- Located on the rear panel between USB-C port and Ethernet
- 5.5mm OD × 2.5mm ID barrel jack
- Center positive polarity

**Alternative Power Option (USB-C PD):**
- The Jetson can also be powered via USB-C Power Delivery (PD) at 15-20V
- Requires USB-C PD power supply with sufficient wattage (100W recommended)
- **Note:** Barrel jack is the recommended method for robotics applications due to reliability and vibration resistance

**Power Mode Reference (at 14.8V battery voltage):**
The Jetson AGX Orin supports multiple power modes via `nvpmodel`:
- **10W mode:** Low power (~0.7A @ 14.8V = 10W)
- **25W mode:** Balanced performance (~1.7A @ 14.8V = 25W)
- **50W mode:** High performance (~3.4A @ 14.8V = 50W)
- **MAXN mode:** Maximum performance (~5.0A @ 14.8V = 74W max per spec)

**Note:** Official spec limits maximum current to 5.0A, which equals 74W at 14.8V (not the full 100W MAXN, but sufficient for robotics applications).

For R2D2 robotics application, you'll likely run in 25-50W mode during active operation.

---

### 2.3 Audio Connections (J511 Header)

#### J511 Audio Header - Complete Pinout

**Official Jetson Nano/AGX Orin Carrier Board J511 Pin Layout:**

```
TOP VIEW (Looking down at the board):

Pin 1 marker (triangle/square) is at top-left

        ╔═══════════════════════════╗
Row A:  ║ 1    3    5    7    9     ║
        ║                            ║
Row B:  ║ 2    4    6    8   10     ║
        ╚═══════════════════════════╝
```

**Complete Pin Assignments (Official Documentation):**

| Pin | Signal Name | Type | Function | R2D2 Usage |
|-----|-------------|------|----------|------------|
| 1 | IN1P | Input | Microphone #1 input | Not used |
| 2 | **AGND** | **GND** | **Audio Ground** | **✅ Connected to PAM8403 GND** |
| 3 | IN2P | Input | Microphone #2 input | Not used |
| 4 | LRCK2/GPIO4/PDM_SDA | Input | Audio dongle detection | Not used |
| **5** | **HPO_R** | **Output** | **Headphone RIGHT channel** | **⚠️ SHOULD USE (correct)** |
| 6 | MIC_IN_DET | Input | Jack/Microphone detect | Not used |
| 7 | SENSE_SEND | NA | Pulled to analog GND | Not used |
| 8 | Key | – | – | – |
| **9** | **HPO_L** | **Output** | **Headphone LEFT channel** | **✅ Currently connected to PAM8403 RIN** |
| 10 | BCLK2/GPIO3/PDM_SCL | Input | – | Not used |

#### Audio Output Signal Chain (Current Wiring)
```
Jetson J511 Pin 9 (HPO_L - I2S Left Channel)
    ↓ (2-wire audio cable)
PAM8403 3W Stereo Amplifier (RIN Input - Right Channel Input)
    ↓ (2-wire speaker cable via R+/R- outputs)
8Ω Speaker (Output)
```

**⚠️ Hardware Note - Channel Mismatch:**

The current wiring connects:
- **Jetson J511 Pin 9 (HPO_L - LEFT channel)** → **PAM8403 RIN (RIGHT channel input)**
- This creates a LEFT→RIGHT channel mismatch

**For correct wiring (future hardware fix):**
```
Jetson J511 Pin 2 (AGND) ──────→ PAM8403 GND (Audio ground reference)
Jetson J511 Pin 5 (HPO_R) ─────→ PAM8403 RIN (RIGHT channel audio)
PAM8403 R+ and R− ─────────────→ 8Ω Speaker
```

**Why This Matters:**
- PAM8403 has TWO separate inputs: **LIN** (Left Input) and **RIN** (Right Input)
- Speaker is connected to **RIGHT outputs (R+/R−)**
- Audio source should match: **RIGHT channel (Pin 5)** → **RIN** → **Right speaker**
- Current setup: LEFT channel (Pin 9) → RIN → causes channel swap

**Current Status:** ✅ Audio works (LEFT channel playing through right speaker path)  
**Future Fix:** Move wire from Pin 9 → Pin 5 for correct channel matching  
**Impact:** Low priority - mono audio works correctly, only matters for stereo content

**J511 Audio Header Pinout Reference:**
| Pin | Signal | Function | Connected To |
|-----|--------|----------|--------------|
| 2 | AGND | Audio Ground | PAM8403 GND ✅ |
| 5 | HPO_R | I2S Right Channel | Should connect to PAM8403 RIN (future fix) |
| 9 | HPO_L | I2S Left Channel | Currently connected to PAM8403 RIN ✅ |

**Configuration:**
- **ALSA Device:** `hw:1,0`
- **Sample Rate:** Variable (typically 44.1kHz or 48kHz)
- **Amplifier Power:** 3W stereo (using left channel only currently)
- **Speaker Impedance:** 8Ω
- **Status:** ✅ Operational
- **Phase:** 1 (Audio feedback), 2 (Speech output)

**Wiring:**
- J511 Pin 9 → PAM8403 Left Input
- PAM8403 Left Output → Speaker (+)
- PAM8403 GND → Speaker (-)

---

### 2.4 GPIO Connections (40-Pin Expansion Header)

#### Current Pin Assignments (Phase 1-2)

| Pin # | Physical | GPIO # | Signal | Function | Wire Color | Connected To | Current (mA) | Status |
|-------|----------|--------|--------|----------|------------|--------------|--------------|--------|
| 1 | 3.3V | — | Power | LED Power | Red | White LED Panel (+) | 20-50 | ✅ |
| 6 | GND | — | Ground | LED Ground | Black | White LED Panel (-) | — | ✅ |
| 22 | GPIO17 | GPIO 17 | Output | LED Control (ON/OFF) | Blue | White LED Panel (Control) | <1 | ✅ |
| 32 | GPIO12 | GPIO 32 | Input | Shutdown Button | — | Momentary Button | — | ✅ |

**White LED Panel Wiring:**
- **Type:** Non-addressable white LED array (16 SMD LEDs)
- **Voltage:** 3V DC (powered from 3.3V pin with small voltage drop)
- **Current:** 20-50mA total (within safe limits for 3.3V pin)
- **Control:** Simple ON/OFF via GPIO 17
- **Wiring:**
  - Red wire → Pin 1 or 17 (3.3V)
  - Blue wire → Pin 22 (GPIO 17)
  - Black wire → Pin 6 (GND)
- **Function:** Visual feedback for person recognition state
  - LED ON = Recognized (RED status)
  - LED OFF = Lost/Unknown (BLUE/GREEN status)

**Shutdown Button (Detailed Specifications):**

**Button Hardware:**
- **Type:** Momentary push button (normally open, NO)
- **Location:** 40-pin expansion header (accessible on robot chassis)
- **Actuation Force:** Typical momentary button (~100-200gf)
- **Contacts:** SPST (Single Pole Single Throw)
- **Ratings:** 50mA @ 12V DC minimum
- **Expected Lifetime:** 100,000+ actuations

**Electrical Connection:**
- **Signal Pin:** Pin 32 (GPIO09 / GPIO32 chipset designation)
- **Ground Pin:** Pin 39 (GND) - physically adjacent to Pin 32
- **Wiring:** 2-wire button cable (any gauge, low current signal)
  - Terminal 1 → Pin 32 (GPIO signal)
  - Terminal 2 → Pin 39 (GND)
- **Pull Resistor:** Internal pull-up enabled in software (GPIO configured as INPUT with PULL_UP)
- **Logic Levels:**
  - Idle (not pressed): GPIO reads HIGH (3.3V via pull-up)
  - Pressed: GPIO reads LOW (0V, shorted to GND)
- **Debounce:** 100ms software debounce (prevents electrical noise false triggers)

**Function & Behavior:**
- **Action:** Press button to initiate graceful system shutdown
- **Command:** Executes `shutdown -h now` via subprocess
- **Response Time:** Immediate detection (<100ms polling interval)
- **Shutdown Sequence:**
  1. Button press detected (falling edge)
  2. 100ms debounce delay
  3. Button release detected (rising edge)
  4. System logs event
  5. Shutdown command issued
  6. System saves state and powers down (~30 seconds)

**Software Implementation:**
- **Service:** `r2d2-powerbutton.service` (systemd, auto-start on boot)
- **Script:** `/usr/local/bin/r2d2_power_button.py`
- **Logging:** `/var/log/r2d2_power_button.log`
- **Polling Rate:** 20ms (50 Hz detection loop)
- **State Detection:** Edge detection (falling + rising) with duration tracking

**Testing & Verification:**
Manual GPIO test:
```bash
python3 << 'EOF'
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print(f"Pin 32 state: {GPIO.input(32)}")  # Should be 1 (HIGH) when not pressed
GPIO.cleanup()
EOF
```

**Status:** ✅ Tested and verified operational (December 9, 2025)
- Button press triggers graceful shutdown
- Debouncing prevents false triggers
- Service auto-starts on boot

#### Reserved Pin Assignments (Phase 3 - Motors) **[PROPOSED]**

| Pin # | Physical | GPIO # | Signal | Function | Device | Status |
|-------|----------|--------|--------|----------|--------|--------|
| 11 | GPIO17 | GPIO 17 | PWM Output | Available for future use | — | — |
| 13 | GPIO27 | GPIO 27 | PWM Output | Available for future use | — | — |
| 15 | GPIO22 | GPIO 22 | PWM Output | Dome Motor PWM | Head Rotation Motor | 💡 Suggested |
| 29 | GPIO5 | GPIO 5 | Digital Output | Dome Motor DIR | Head Rotation Motor | 💡 Suggested |
| 31 | GPIO6 | GPIO 6 | Digital Output | Dome Motor EN | Head Rotation Motor | 💡 Suggested |
| 33 | GPIO13 | GPIO 13 | PWM Output | Left Wheel PWM | Left Motor Driver | 💡 Suggested |
| 35 | GPIO19 | GPIO 19 | Digital Output | Left Wheel DIR | Left Motor Driver | 💡 Suggested |
| 37 | GPIO26 | GPIO 26 | Digital Output | Left Wheel EN | Left Motor Driver | 💡 Suggested |
| 36 | GPIO16 | GPIO 16 | Digital Output | Right Wheel DIR | Right Motor Driver | 💡 Suggested |
| 38 | GPIO20 | GPIO 20 | Digital Output | Right Wheel EN | Right Motor Driver | 💡 Suggested |

**Note:** Pin assignments marked as 💡 are proposed designs pending Phase 3 implementation. Encoder inputs not yet allocated (would require 6 additional GPIO pins for 3 encoders × 2 channels).

#### Reserved Pin Assignments (Phase 4 - LED Strip) **[PROPOSED]**

| Pin # | Physical | GPIO # | Signal | Function | Device | Status |
|-------|----------|--------|--------|----------|--------|--------|
| 12 | GPIO18 | GPIO 18 | PWM/SPI Output | WS2812B Data | RGB LED Strip | 💡 Idea |

**Jetson AGX Orin 40-Pin GPIO Pinout Reference:**

![Jetson AGX Orin 40-pin header](https://developer.download.nvidia.com/embedded/images/jetsonAgxOrin/getting_started/jaodk_40pin_header_pinout.png)

*Image source: [NVIDIA Jetson AGX Orin Developer Kit User Guide](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html)*

**Pin identification:** Look for the white triangle marking on the PCB to identify Pin 1.

---

### 2.5 Automation Header (J42)

#### J42 Automation Header - Complete Pinout

**Header Location:** J42 on Jetson AGX Orin carrier board (labeled)

**Complete Pin Assignments:**

| Pin | Signal | Type | Function | R2D2 Usage |
|-----|--------|------|----------|------------|
| 1 | GND | Ground | System ground | ✅ Boot/wake button ground |
| 2 | PWR_BTN | Input | Hardware power button (not used) | ⚠️ Do NOT use - hardware only |
| 3 | Reserved | — | — | Not connected |
| 4 | POWER | Input/Output | Software power control | ✅ Boot/wake button signal |

**⚠️ CRITICAL: Pin 2 vs Pin 4 Distinction:**

- **Pin 2 (PWR_BTN):** Hardware-only power button signal
  - Directly connected to PMIC (Power Management IC)
  - Cannot be controlled by software
  - Used for physical power button on dev kit
  - **NOT SUITABLE** for software-controlled boot/wake

- **Pin 4 (POWER):** Software-controlled power signal
  - Can be triggered programmatically
  - Used for remote boot/wake functionality
  - **CORRECT PIN** for R2D2 boot/wake button
  - Provides clean software integration

#### Boot/Wake Button (Detailed Specifications)

**Button Hardware:**
- **Type:** Momentary push button (normally open, NO)
- **Location:** J42 Automation Header (4-pin header on carrier board)
- **Actuation:** Momentary contact (press and release)
- **Contacts:** SPST (Single Pole Single Throw)
- **Ratings:** Low voltage signal (~3.3V logic level)

**Electrical Connection:**
- **Signal Pin:** Pin 4 (POWER) - software-controllable power signal
- **Ground Pin:** Pin 1 (GND) - header ground reference
- **Wiring:** 2-wire button cable
  - Terminal 1 → Pin 4 (POWER signal)
  - Terminal 2 → Pin 1 (GND)
- **Logic:** Active-low trigger (short to ground activates)
- **Pull Resistor:** Internal pull-up on POWER pin (hardware)

**Function & Behavior:**
- **Action:** Short Pin 4 (POWER) to Pin 1 (GND) to trigger wake/boot
- **Use Cases:**
  1. Wake Jetson from low-power state (suspend/sleep)
  2. Boot Jetson from complete shutdown (if power still applied)
  3. Remote power-on capability
- **Response:** Hardware-level power management (instant response)
- **Duration:** Brief contact sufficient (~100ms minimum)

**Hardware-Level Operation:**
When button is pressed (pins shorted):
1. POWER pin pulled to GND
2. Jetson power management detects signal
3. Power-on sequence initiated
4. System boots normally
5. No software intervention required (hardware-triggered)

**Wiring Diagram:**
```
Boot/Wake Button (Momentary)
    Terminal 1 ──→ J42 Pin 4 (POWER)
    Terminal 2 ──→ J42 Pin 1 (GND)

Press button → Pins 1 & 4 shorted → Wake/boot triggered
```

**Testing Procedure:**
1. Shutdown Jetson completely: `sudo shutdown -h now`
2. Wait for system to fully power down (~30 seconds)
3. Press and release boot/wake button
4. Observe system boot sequence initiation
5. Verify normal boot to login prompt

**Status:** ⏳ Ready for testing (wired but not yet verified)
- Hardware installed and wired correctly
- Pin 4 (POWER) confirmed as correct choice
- Awaiting first test after shutdown

**Safety Notes:**
- Safe to press during operation (no adverse effects)
- Does not force immediate shutdown (use shutdown button for that)
- No software driver required (hardware-level function)
- Compatible with remote power management systems

---

## 3. Component Catalog (Detailed Specifications)

### 3.1 Compute Platform

#### NVIDIA Jetson AGX Orin 64GB Developer Kit
- **Part Number:** 945-13730-0050-000
- **Architecture:** ARM64 (aarch64)
- **CPU:** 12-core ARM Cortex-A78 @ 2.4 GHz
- **GPU:** 2048-core NVIDIA Ampere (504 CUDA cores equivalent)
- **Memory:** 64GB LPDDR5X (275 GB/s bandwidth)
- **Storage:** Internal eMMC (~32GB usable after OS)
- **Power:** Variable TDP 15-100W (power mode configurable)
- **Operating System:** Ubuntu 22.04 Jammy (L4T - Linux for Tegra)
- **Kernel:** Linux 5.15.148-tegra
- **JetPack Version:** 6.2.1 (L4T R36.4)
- **CUDA Version:** 12.6
- **ROS 2 Version:** Humble Hawksbill
- **Status:** ✅ Mounted & Running
- **Phase:** 1 (Foundation)

**Key Interfaces:**
- USB 3.0 ports (multiple)
- 40-pin GPIO expansion header
- J511 audio header (I2S)
- J42 automation header
- Gigabit Ethernet
- USB-C device port (for USB networking @ 192.168.x.1)

---

### 3.2 Chassis & Mechanical

#### DeAgostini R2-D2 1:2 Scale Kit
- **Scale:** 1:2 (half-size of movie prop)
- **Height:** 48 cm
- **Material:** Plastic body panels, metal frame
- **Includes:**
  - 2× DC motors for legs (with integrated encoders)
  - 1× DC motor for dome (head rotation, with encoder)
  - Drive mechanism and gearing
  - Body panels and structural components
- **Modifications:** Custom internal mounting for Jetson and peripherals
- **Status:** ✅ Complete
- **Phase:** 1 (Mechanical foundation)

---

### 3.3 Vision System

#### Luxonis OAK-D Lite Auto Focus
- **Model:** OAK-D Lite AF
- **Serial Number:** 19443010E1D30C7E00 (this specific unit)
- **Processor:** Intel Movidius MyriadX
- **RGB Camera:**
  - Sensor: 4K capable (using 1920×1080 mode)
  - Actual resolution: 1280×1080 (with padding to 1920×1080)
  - Frame rate: 30 FPS
  - Auto-focus: Yes
- **Stereo Depth:**
  - Sensors: 2× OV9782 (1MP grayscale)
  - Depth range: 0.2m - 10m (typical)
  - Depth output: Disparity map or depth map
- **IMU:** BMI270 (6-axis accelerometer + gyroscope)
- **Interface:** USB 3.0 (USB-C connector)
- **Power:** Bus-powered (500mA @ 5V from Jetson USB)
- **SDK:** DepthAI 2.31.0.0
- **Connection Requirement:** MUST be direct to Jetson (not through USB hub)
- **Status:** ✅ Integrated & Operational
- **Phase:** 1 (Perception)

**Current Usage:**
- RGB stream published to `/oak/rgb/image_raw` (30 Hz)
- Depth and IMU features available but not yet utilized

---

### 3.4 Audio System

#### Input: HyperX QuadCast S USB Microphone
- **Model:** HyperX QuadCast S
- **Type:** Condenser microphone (USB audio class)
- **Polar Pattern:** Cardioid (default), also supports stereo, omnidirectional, bidirectional
- **Sample Rate:** 48kHz (native)
- **Bit Depth:** 16-bit
- **Channels:** Stereo (2-channel)
- **Interface:** USB Type-A (plug-and-play)
- **Power:** USB bus-powered
- **LED Indicator:** RGB lighting (status indicator)
- **Mute Button:** Hardware mute with LED feedback
- **Auto-Detection:** Searched by device name ("hyperx" or "quadcast")
- **Usage:** Resampled to 24kHz mono for OpenAI Realtime API
- **Status:** ✅ Working
- **Phase:** 2 (Speech)

#### Output: PAM8403 Amplifier + Speaker
- **Amplifier Model:** PAM8403 Digital Amplifier Module
- **Power Output:** 3W × 2 (stereo), using left channel
- **Supply Voltage:** 2.5V - 5.5V DC
- **Efficiency:** Class D (high efficiency, low heat)
- **Input:** Analog audio or I2S digital
- **Connection:** I2S from Jetson J511 Pin 9 (HPO_L)
- **Speaker:** 8Ω impedance
- **Power Source:** External (not from Jetson GPIO)
- **ALSA Device:** `hw:1,0`
- **Status:** ✅ Operational
- **Phase:** 1 (Audio alerts), 2 (Speech output)

**Signal Chain:**
```
OpenAI Realtime API TTS → speech_node → ALSA (hw:1,0) → 
I2S (J511 Pin 9) → PAM8403 Amplifier → 8Ω Speaker
```

---

### 3.5 Motor & Drive System (Phase 3 - Future)

#### Wheel Motors (2×)
- **Model:** DeAgostini DC Motors (from kit)
- **Type:** Brushed DC motor with integrated encoder
- **Quantity:** 2 (left wheel, right wheel)
- **Voltage:** 12-14.8V (compatible with 4S LiPo)
- **Encoder Type:** Quadrature encoder (2-channel)
- **Encoder Resolution:** TBD (to be measured)
- **Gear Ratio:** TBD (part of DeAgostini drivetrain)
- **Control:** Pololu MC33926 H-bridge drivers
- **Status:** ⏳ Not yet integrated (motors assembled in chassis)
- **Phase:** 3 (Navigation)

#### Dome Motor (Head Rotation)
- **Model:** DeAgostini DC Motor (from kit)
- **Type:** Brushed DC motor with integrated encoder
- **Quantity:** 1
- **Function:** Z-axis head rotation (dome/head spinning)
- **Voltage:** 12-14.8V
- **Encoder Type:** Quadrature encoder (2-channel)
- **Control:** Motor driver (TBD - could reuse MC33926 or similar)
- **Status:** ⏳ Not yet integrated
- **Phase:** 3 (Expression & tracking)

#### Motor Drivers (Pololu G2 High-Power 24v21)
- **Model:** [Pololu G2 High-Power Motor Driver 24v21](https://www.pololu.com/product/2995)
- **Part Number:** 2995
- **Quantity:** TBD (1-3 units depending on motor configuration)
- **Type:** Discrete MOSFET H-bridge (single-channel)
- **Board Size:** 1.3″ × 0.8″ (33mm × 20mm)
- **Voltage Range:**
  - Operating: 6.5V to 40V (absolute maximum)
  - Recommended maximum: 34V (accounting for voltage ripple)
  - **Max nominal battery:** 28V (safe for 4S LiPo @ 16.8V charged)
- **Current Capacity:**
  - Continuous: 21A per driver (no heatsink required)
  - Peak (with current limiting): 50A default threshold (adjustable)
- **Logic Inputs:**
  - Compatible with 1.8V, 3.3V, 5V logic (✅ Jetson GPIO compatible)
  - Control modes: Sign-magnitude or locked-antiphase
  - Minimum I/O lines: 2 per driver (DIR + PWM)
- **Features:**
  - PWM frequency: Up to 100kHz
  - Current sense output: ~20mV/A (proportional to motor current)
  - Active current limiting (chopping)
  - Reverse-voltage protection
  - Undervoltage shutdown
  - Short circuit protection
  - **Note:** No over-temperature protection (external monitoring recommended)
- **Control Interface:**
  - DIR (direction): Digital input
  - PWM (speed): PWM input (0-100% duty cycle)
  - SLP (sleep): Optional low-power mode
  - FLT (fault): Output signal (LOW = fault detected)
  - CS (current sense): Analog output (~20mV/A)
- **Status:** ✅ Assembled, not wired
- **Phase:** 3 (Navigation)
- **Price:** $56.95 USD (reference)

**Usage Plan:**
- **Driver #1:** Left wheel motor (21A continuous capacity)
- **Driver #2:** Right wheel motor (21A continuous capacity)
- **Driver #3 (optional):** Dome motor (21A capacity, likely overkill for dome)

**Alternative:** Could use dual-channel version for Arduino/Raspberry Pi form factor (see related products on [Pololu's website](https://www.pololu.com/product/2995))

#### Camera Orientation (Dome Motor Provides Pan)
- **Solution:** OAK-D camera is mounted inside the R2-D2 dome
- **Pan (Horizontal Rotation):** Dome motor rotates entire head assembly
  - **Motor:** DeAgostini DC motor with encoder (from kit)
  - **Driver:** Pololu G2 24v21 #3 (optional third driver)
  - **Range:** 360° continuous rotation
- **Tilt (Vertical):** Not needed for R2-D2 application
  - Camera is fixed at optimal height/angle within dome
  - R2-D2 design does not include head tilt mechanism
- **Status:** ✅ No additional servos required
- **Phase:** 3 (Navigation - dome motor integration)

**Advantages of Dome Motor Solution:**
- ✅ No additional servos needed (saves cost and complexity)
- ✅ Authentic R2-D2 appearance (dome rotation is canonical)
- ✅ Sufficient for navigation and person tracking
- ✅ Uses existing DeAgostini hardware (no additional components)

---

### 3.6 Power System

#### Main Battery (3× Turnigy 4S LiPo)
- **Model:** Turnigy 4S LiPo (HobbyKing)
- **Quantity:** 3× batteries (can be used individually or in parallel)
- **Capacity per battery:** Typically 3000-5000mAh (verify specific model)
- **Voltage:** 14.8V nominal (16.8V fully charged, 14.0V cutoff per battery)
- **Configuration:** 4 cells in series (4S) per battery
- **Connector:** XT60 (high-current connector, standard on Turnigy)
- **Discharge Rate:** Typically 20C-40C (verify specific model - critical for motor loads)
- **Chemistry:** LiPo (requires balance charging and voltage monitoring)
- **Status:** ✅ Charged & Ready
- **Phase:** 1 (Power foundation)
- **Source:** HobbyKing (hobbyking.com)

**Usage Options:**
1. **Single battery mode:** Use 1× battery for testing/light loads (runtime: ~1-2 hours)
2. **Parallel mode:** Connect 2-3× batteries in parallel for extended runtime and higher current capacity
   - **Parallel connection:** Requires balance charging adapter or parallel board
   - **Benefit:** 2× batteries = 2× capacity and 2× current capability
   - **Runtime estimate:** 2× batteries = ~2-4 hours, 3× batteries = ~3-6 hours

**Power Distribution:**
- Powers all system components directly or via DC-DC converters
- **Direct 14.8V connections:**
  - Motor drivers (Pololu G2 24v21 - accepts 6.5-40V)
  - **Jetson AGX Orin (accepts 7-20V) ✅ No converter needed!**
- Stepped down to 5V for servos and peripherals only

**Critical Note:** Turnigy batteries from HobbyKing typically have excellent discharge ratings. Verify C-rating on your specific model to ensure it can handle peak motor current draw (~13A total system, ~25A+ for parallel motor operation).

#### DC-DC Converters

**12V Buck Converter (Jetson Power)**
- **Input:** 14.8V nominal (from 3× Turnigy 4S LiPo in parallel)
  - Range: 14.0V (cutoff) to 16.8V (fully charged)
- **Output:** 12V DC ±5% (11.4V - 12.6V acceptable)
  - **Note:** Jetson accepts 9V-20V DC, but 12V is recommended nominal
- **Current Capacity:** Must support ≥8.5A continuous (for 100W peak)
  - Recommended: 10A+ rated converter (provides safety margin)
- **Type:** Buck (step-down) switching converter with current limiting
- **Output Connector:** DC barrel jack (5.5mm OD × 2.1mm ID, center-positive)
- **Suggested Models:**
  - DROK LM2596 (3A, insufficient - NOT recommended)
  - Pololu D36V50F12 (5A @ 12V, marginal)
  - **Recommended:** XL4015 or LM2596HV-based (≥10A capacity)
  - Alternative: DROK 15A adjustable buck converter

**Jetson AGX Orin Power Requirements (Official NVIDIA Specs):**
- **Input Voltage Range:** 9V - 20V DC
- **Nominal Voltage:** 12V DC (recommended)
- **Current Draw:**
  - Idle/Low Power: ~2-3A @ 12V (24-36W)
  - Typical Operation: 4-6A @ 12V (48-72W)  
  - Peak Performance (MAXN mode): 8.3A @ 12V (100W)
- **Power Connector:** J16 DC barrel jack on Jetson
  - Outer Diameter: 5.5mm
  - Inner Diameter: 2.1mm
  - Polarity: Center-positive (+), Sleeve negative (-)
- **Power Modes (nvpmodel):**
  - 10W mode: ~1A @ 12V
  - 15W mode: ~1.5A @ 12V
  - 30W mode: ~3A @ 12V
  - 50W mode: ~5A @ 12V
  - MAXN mode: ~8.3A @ 12V (unrestricted performance)

**Reference:** [NVIDIA Jetson AGX Orin Developer Kit Layout & Power Supply](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html)

**Connection Diagram (Battery to Jetson):**
```
3× Turnigy 4S LiPo (14.8V, parallel) → Main Power Bus (14.8V)
    │
    ├─→ 10A Fuse (protection)
    │
    └─→ Buck Converter Input (14.8V)
        │ (Step-down conversion)
        ├─→ Output Capacitor (1000-2200µF, stability)
        │
        └─→ DC Barrel Jack (5.5mm × 2.1mm, center-positive)
            │ (14-16 AWG wire, handles 10A)
            │
            └─→ Jetson AGX Orin J16 Power Input
```

**Safety & Wiring:**
- **Fuse:** 10A fuse on buck converter input (protects from short circuits)
- **Wire Gauge:** 
  - Input (14.8V): 14 AWG minimum (handles 15A safely)
  - Output (12V): 14-16 AWG (handles 10A safely)
- **Capacitor:** 1000-2200µF electrolytic on buck output (reduces voltage ripple)
- **Heatsink:** Buck converter may require heatsinking at high loads (>50W)
- **Monitoring:** Use `tegrastats` to monitor Jetson power consumption
- **Connector Output:** 
  - **Recommended:** Barrel jack (5.5mm OD, 2.1mm ID, center positive) → Jetson J16
  - **Alternative:** Screw terminals → Wire to Jetson J16 barrel jack
- **Efficiency:** ~85-95% (typical for buck converters)
  - Input current: ~7A @ 14.8V (for 100W output accounting for losses)

**Jetson Power Connector Location (Official NVIDIA Reference):**
- **Power Input:** J16 DC barrel jack on Jetson AGX Orin carrier board
- **Connector Specs:** 5.5mm OD × 2.1mm ID, center-positive
- **Visual Reference:** [NVIDIA Jetson AGX Orin Developer Kit Layout](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html)

**Key Jetson Connectors (from NVIDIA documentation):**
- **J16:** DC Power Input (9-20V, recommended 12V)
- **J511:** Audio Header (Pin 9 = HPO_L for I2S audio output)
- **J42:** Automation Header (Pin 4 = POWER, Pin 1 = GND for boot/wake)
- **J14:** 40-pin GPIO expansion header (LED, buttons, motor control signals)
- **USB Ports:** Multiple USB 3.0 Type-A ports for camera and microphone

**Image Attribution:** Connector layout diagrams and specifications from NVIDIA's official Jetson AGX Orin Developer Kit User Guide, used per NVIDIA developer documentation license
- **Status:** ✅ Operational
- **Phase:** 1 (Compute power)

**Jetson Power Input Options (see NVIDIA Developer Kit Layout below):**
1. **J507 Barrel Jack (Recommended):** 5.5mm × 2.1mm barrel jack, 9V-20V input, center positive
2. **J508 DC Power Header:** 2-pin header (VIN + GND), alternative to barrel jack
3. **USB-C (Not recommended for robot):** 5V/3A (15W max) - insufficient for full performance

**5V Buck Converter (Peripherals & Servos)**
- **Input:** 14.8V (from 4S LiPo battery)
- **Output:** 5V DC
- **Current Capacity:** TBD (must support servos ~2A + peripherals ~1A = 3A minimum)
- **Type:** Buck (step-down) converter
- **Usage:** Powers RC servos, future 5V peripherals
- **Status:** ⏳ Partial (converter available, not fully wired)
- **Phase:** 3 (Servo power)

**Power Distribution Strategy (Simplified - No Buck Converter for Jetson!):**
```
3× Turnigy 4S LiPo (14.8V, ~15Ah total in parallel, ~222Wh)
    │
Power Distribution Board
    │
    ├─→ [14.8V High-Current Rail] → Pololu G2 Motor Drivers (21A each)
    │   ├─→ Driver #1 → Left Wheel Motor (DeAgostini DC)
    │   ├─→ Driver #2 → Right Wheel Motor (DeAgostini DC)
    │   └─→ [Future Driver #3] → Dome Motor (DeAgostini DC)
    │
    ├─→ [14.8V Direct + 10A Fuse] → Jetson AGX Orin DC Jack (7-20V spec, 5.0A max)
    │   └─→ No converter needed! ✅ Simpler & more efficient
    │
    └─→ [5V Buck Converter] (3A) → PAM8403 Amplifier + WS2812B LEDs (Phase 4)
```

---

### 3.7 Status Indicators

#### Current: White LED Panel
- **Type:** Non-addressable white LED array
- **LEDs:** 16 SMD LEDs (surface-mount)
- **Voltage:** 3V DC nominal (powered from Jetson 3.3V with small drop)
- **Current:** 20-50mA total (all LEDs in parallel/series configuration)
- **Control:** Simple ON/OFF via GPIO 17 (Pin 22)
- **Connector:** 3-wire cable
  - Red wire → Pin 1 or 17 (3.3V power)
  - Blue wire → Pin 22 (GPIO 17 control signal)
  - Black wire → Pin 6 (GND)
- **Function:** Visual feedback for person recognition status
  - LED ON = Recognized person present (RED status)
  - LED OFF = Person lost or unknown (BLUE/GREEN status)
- **Status:** ✅ Operational
- **Phase:** 1 (Status feedback)

**Technical Notes:**
- Current draw within safe limits for Jetson 3.3V pin (max 50mA)
- No current-limiting resistor needed (built into LED panel)
- GPIO 17 can safely sink/source the control signal current

#### Future: WS2812B RGB LED Strip (Phase 4) 💡
- **Type:** Addressable RGB LED strip
- **Model:** WS2812B (or compatible: SK6812, WS2813)
- **LEDs:** ~24-30 individual addressable LEDs
- **Voltage:** 5V DC
- **Current:** ~50-60mA per LED at full white (max: 1.5-1.8A for 30 LEDs)
- **Control Protocol:** Serial one-wire protocol (precise timing required)
- **Suggested Connection:**
  - Data pin → Pin 12 (GPIO 18) - PWM/SPI capable
  - Power → 5V rail (from DC-DC converter)
  - Ground → GND
- **Control Library:** `rpi_ws281x` or `adafruit-circuitpython-neopixel` (Jetson compatible)
- **Function:** Advanced status patterns, personality expression, ambient lighting
- **Status:** ⏳ Reserved for Phase 4
- **Phase:** 4 (Personality & expression)

**Design Considerations:**
- Requires precise timing (hardware PWM or SPI recommended)
- High current draw requires external 5V power (not from GPIO)
- Add 300-500Ω resistor on data line for signal integrity
- Add 1000µF capacitor across power supply for current spikes

---

### 3.8 Control Inputs (Power Management)

The R2D2 system provides two physical buttons for power management, enabling safe shutdown and reliable boot/wake functionality. This dual-button design ensures system safety while providing convenient power control.

#### Button 1: Shutdown Control (Software-Triggered)

**Hardware Specifications:**
- **Type:** Momentary push button (normally open, SPST)
- **Connection:** 40-pin GPIO header
  - Pin 32 (GPIO09/GPIO32) - Signal input
  - Pin 39 (GND) - Ground reference
- **Actuation:** Light touch, momentary contact
- **Wiring:** 2-wire cable (low current signal)
- **Pull Configuration:** Internal pull-up enabled (software-configured)
- **Electrical:** 3.3V logic HIGH (idle), 0V logic LOW (pressed)

**Software Implementation:**
- **Service:** `r2d2-powerbutton.service` (systemd, enabled, auto-start)
- **Script Location:** `/usr/local/bin/r2d2_power_button.py` (deployed)
- **Source Location:** `/home/severin/dev/r2d2/r2d2_power_button_simple.py` (development)
- **Logging:** `/var/log/r2d2_power_button.log` (automatic rotation via systemd)
- **Class:** `PowerButtonHandler` (~150 lines Python)

**Operational Behavior:**
- **Detection Method:** GPIO polling (20ms interval, 50 Hz sampling rate)
- **Debouncing:** 100ms threshold (prevents electrical noise false triggers)
- **Edge Detection:** Falling edge (press) + rising edge (release) tracked
- **Duration Tracking:** Press duration logged but not required for activation
- **Action:** Any valid press triggers shutdown (no hold time requirement)
- **Command:** Executes `shutdown -h now` via subprocess
- **Response Time:** Immediate (<100ms from press to action)

**Shutdown Sequence:**
1. User presses button
2. GPIO detects falling edge (HIGH → LOW transition)
3. 100ms debounce timer starts
4. GPIO state confirmed stable
5. User releases button
6. GPIO detects rising edge (LOW → HIGH transition)
7. Event logged: "Button pressed" + "Button released (duration: X.XXs)"
8. Shutdown command issued: "Initiating shutdown..."
9. System gracefully saves state and powers down (~30 seconds)

**Service Management:**
```bash
# Check service status
sudo systemctl status r2d2-powerbutton.service

# View real-time logs
journalctl -u r2d2-powerbutton.service -f

# View persistent log file
tail -50 /var/log/r2d2_power_button.log

# Restart service (if needed)
sudo systemctl restart r2d2-powerbutton.service
```

**Testing Results:**
- **Test Date:** December 9, 2025, 07:30:40
- **Result:** ✅ PASS - Single button press triggered graceful shutdown
- **Log Excerpt:**
  ```
  07:30:40 Button pressed
  07:30:41 Button released (duration: 0.32s)
  07:30:41 Initiating shutdown...
  07:30:41 ACTION: Shutting down system...
  ```
- **Verification:** System shutdown gracefully, service auto-restarted on next boot

**Status:** ✅ Tested and Operational
- **Phase:** 1 (System control)
- **Reliability:** 100% success rate in testing
- **Auto-start:** Enabled on boot
- **Auto-restart:** Service recovers automatically on failure

#### Button 2: Boot/Wake Control (Hardware-Triggered)

**Hardware Specifications:**
- **Type:** Momentary push button (normally open, SPST)
- **Connection:** J42 Automation Header (4-pin header)
  - Pin 4 (POWER) - Software-controlled power signal
  - Pin 1 (GND) - Ground reference
- **Actuation:** Brief contact (minimum ~100ms)
- **Wiring:** 2-wire cable
- **Pull Configuration:** Internal pull-up (hardware, on POWER pin)
- **Trigger Method:** Active-low (short to ground activates)

**Function:**
- **Primary:** Wake Jetson from low-power/suspend state
- **Secondary:** Boot Jetson from complete shutdown (if power still applied)
- **Trigger Level:** Hardware-level power management (no software required)
- **Response:** Immediate (power management IC handles wake sequence)

**Operational Behavior:**
- **Action:** Short Pin 4 (POWER) to Pin 1 (GND)
- **Duration:** Brief press sufficient (~100ms minimum)
- **Detection:** Hardware power management IC (PMIC)
- **Sequence:**
  1. Button pressed (pins shorted)
  2. POWER signal pulled to GND
  3. PMIC detects wake signal
  4. Power-on sequence initiated
  5. System boots normally
  6. Button can be released anytime after ~100ms

**Use Cases:**
1. Wake from sleep/suspend modes
2. Boot from shutdown (if main power still connected)
3. Remote power-on (via relay or remote button)
4. Recovery from low-power states

**Testing Status:** ⏳ Ready for Testing
- **Wiring:** Completed and verified
- **Pin Selection:** Confirmed correct (Pin 4 POWER, not Pin 2 PWR_BTN)
- **Hardware Test:** Pending first shutdown cycle test
- **Expected Result:** System should boot when button pressed after shutdown

**Important Notes:**
- **Pin 2 vs Pin 4:** This system uses Pin 4 (POWER), NOT Pin 2 (PWR_BTN)
  - Pin 2 (PWR_BTN) is hardware-only, cannot be software-controlled
  - Pin 4 (POWER) is correct for software-controlled boot/wake
- **No Software Driver:** This is a hardware function, no service required
- **Safe to Test:** Pressing during operation causes no harm
- **Power Requirement:** Main battery/power must remain connected

**Phase:** 1 (System control)  
**Documentation:** See Section 2.5 (Automation Header) for wiring details

---

**Power Control System Summary:**

| Feature | Button 1 (Shutdown) | Button 2 (Boot/Wake) |
|---------|---------------------|---------------------|
| **Function** | Graceful shutdown | Wake from shutdown/sleep |
| **Connection** | Pin 32 + GND (40-pin) | J42 Pin 4 + Pin 1 |
| **Trigger** | Software (systemd service) | Hardware (PMIC) |
| **Response** | Immediate shutdown | Immediate boot/wake |
| **Status** | ✅ Operational | ⏳ Ready to test |
| **Testing** | Verified Dec 9, 2025 | Awaiting first test |

**For detailed troubleshooting and service configuration, see:** [`005_SYSTEMD_SERVICES_REFERENCE.md`](005_SYSTEMD_SERVICES_REFERENCE.md) Section on r2d2-powerbutton service

---

## 4. Pin Allocation Summary

### 4.1 Currently Used (Phase 1-2) ✅

**USB Ports:**
- USB 3.0 Port 1: OAK-D Lite Camera
- USB Port 2: HyperX QuadCast S Microphone

**J511 Audio Header:**
- Pin 9 (HPO_L): I2S audio output to PAM8403

**J42 Automation Header:**
- Pin 1 (GND) + Pin 4 (POWER): Boot/Wake button

**40-Pin GPIO Header:**
- Pin 1 or 17 (3.3V): LED power
- Pin 6 (GND): LED ground
- Pin 22 (GPIO17): LED control
- Pin 32 (GPIO32): Shutdown button

**Total Pins Used:** 7 (4 GPIO pins + 1 audio + 2 automation)

---

### 4.2 Reserved for Phase 3 (Motors & Servos) 💡

**Estimated GPIO Requirements:**

| Function | Pins Required | Suggested Assignment |
|----------|---------------|---------------------|
| Left wheel motor control | 3 (PWM, DIR, EN) | Pins 33, 35, 37 |
| Right wheel motor control | 3 (PWM, DIR, EN) | Pins TBD*, 36, 38 |
| Left wheel encoder | 2 (A, B channels) | Pins TBD |
| Right wheel encoder | 2 (A, B channels) | Pins TBD |
| Dome motor control | 3 (PWM, DIR, EN) | Pins 15, 29, 31 |
| Dome encoder | 2 (A, B channels) | Pins TBD |
| **Total** | **17 pins** | — |

**Notes:**
- *Pin 32 conflict (currently shutdown button, needed for right wheel PWM)
- **Pin 11/GPIO17 conflict (currently LED control, could move to software-only)
- Encoder pins not yet assigned (need 6 additional GPIO inputs)
- May require I2C PWM controller (PCA9685) to reduce pin count

**Alternative Strategy:**
- Use I2C PWM controller for servos (saves 2 GPIO pins)
- Use dedicated motor controller with I2C/UART interface (saves many GPIO pins)
- Move LED to RGB strip (WS2812B on single data pin)

---

### 4.3 Reserved for Phase 4 (RGB LED) 💡

**GPIO Requirements:**

| Function | Pins Required | Suggested Assignment |
|----------|---------------|---------------------|
| WS2812B data line | 1 (PWM/SPI) | Pin 12 (GPIO18) |
| **Total** | **1 pin** | — |

---

### 4.4 Pin Conflict Analysis & Resolution Strategy

**Current Conflicts:**
1. **Pin 32 (GPIO32):** Currently shutdown button, proposed for right wheel motor PWM
2. **Pin 11 (GPIO17):** Currently LED control, proposed for pan servo
3. **Insufficient pins:** 17 pins needed for Phase 3, limited availability after Phase 1-2

**Proposed Resolutions:**
1. Keep shutdown button on Pin 32 (critical safety function)
2. Move right wheel motor PWM to alternative pin (e.g., Pin 13/GPIO27 now available)
3. Migrate LED control to WS2812B RGB strip (Phase 4) freeing Pin 11
4. Use I2C PWM controller (PCA9685) for servos → saves 2 GPIO pins
5. Consider motor controller with I2C/UART interface (e.g., RoboClaw) → saves 6+ GPIO pins

**Final recommendation:** Will be determined during Phase 3 planning based on actual component selection and integration requirements.

---

## 5. Power Budget

### 5.1 Current Power Consumption (Phase 1-2) ✅

| Component | Voltage | Current (Typical) | Current (Peak) | Power (W) | Source |
|-----------|---------|-------------------|----------------|-----------|--------|
| Jetson AGX Orin | 12V | 2-4A (24-48W) | 8.3A (100W) | 24-100W | 12V DC-DC |
| OAK-D Camera | 5V | 500mA | 500mA | 2.5W | USB (Jetson) |
| HyperX Microphone | 5V | 100mA | 200mA | 0.5-1W | USB (Jetson) |
| White LED Panel | 3.3V | 20-50mA | 50mA | 0.07-0.17W | Jetson GPIO |
| PAM8403 Amplifier | 5V | 50-200mA | 500mA | 0.25-2.5W | External |
| **Total (typical)** | — | — | — | **~27-52W** | — |
| **Total (peak)** | — | — | — | **~106W** | — |

**Battery Life Estimate (Phase 1-2):**
- Battery: 5000mAh @ 14.8V = 74Wh
- Typical consumption: ~35W average
- **Estimated runtime: ~2.1 hours** (conservative, accounting for efficiency losses)

---

### 5.2 Projected Power Consumption (Phase 3 - Motors) 💡

| Component | Voltage | Current (Typical) | Current (Peak) | Power (W) | Source |
|-----------|---------|-------------------|----------------|-----------|--------|
| Left wheel motor | 14.8V | 1-2A | 5A | 15-30W | Battery direct |
| Right wheel motor | 14.8V | 1-2A | 5A | 15-30W | Battery direct |
| Dome motor | 14.8V | 0.5-1A | 3A | 7-15W | Battery direct |
| **Motors subtotal (typical)** | — | — | — | **~40W** | — |
| **Motors subtotal (peak)** | — | — | — | **~85W** | — |

**Total System (Phase 3):**
- Typical: ~75W (Jetson + peripherals + motors cruising)
- Peak: ~190W (Jetson max + all motors stalled)

**Battery Life Estimate (Phase 3):**
- Battery: 5000mAh @ 14.8V = 74Wh
- Typical consumption: ~75W average
- **Estimated runtime: ~1 hour** (active navigation)

**Note:** Peak power draw (190W) exceeds battery safe discharge rate. Requires verification of battery C-rating and potential current limiting in motor control.

---

### 5.3 Power Distribution Architecture

```
4S LiPo Battery (14.8V, 5000mAh, 74Wh)
    │
    ├─→ [14.8V Rail - High Current]
    │   ├─→ Pololu MC33926 Driver #1 → Left Wheel Motor (5A peak)
    │   ├─→ Pololu MC33926 Driver #1 → Right Wheel Motor (5A peak)
    │   └─→ Motor Driver (TBD) → Dome Motor (3A peak)
    │
    ├─→ [12V Buck Converter] (8.3A capacity)
    │   └─→ Jetson AGX Orin (2-8A, 24-100W)
    │
    └─→ [5V Buck Converter] (3A capacity)
        ├─→ PAM8403 Amplifier (0.5A peak)
        └─→ WS2812B LED Strip (1.5A peak, Phase 4)
```

**Design Considerations:**
- Battery must support C-rating for peak current draw (~13A total)
- DC-DC converters need adequate heatsinking
- Fuses/current limiting recommended on each power rail
- Voltage monitoring for battery health (LiPo cutoff at 14.0V)

---

## 6. Bill of Materials Summary

### 6.1 Cost Breakdown

| Category | Components | Approximate Cost (USD) |
|----------|------------|------------------------|
| **Compute** | Jetson AGX Orin 64GB | $1,000 |
| **Vision** | OAK-D Lite Camera | $150 |
| **Audio** | HyperX QuadCast S + PAM8403 + Speaker | $150 |
| **Power** | 3× Turnigy 4S LiPo + DC-DC Converters + Harness | $150 |
| **Motor Drivers** | 2-3× Pololu G2 24v21 @ $57 each | $115-170 |
| **Chassis** | DeAgostini R2-D2 Kit (with motors) | $1,400 |
| **Peripherals** | LEDs, buttons, cables, misc | $140 |
| **Total (Compute + Sensors)** | — | **~$2,305-2,360** |
| **Total (Complete System)** | — | **~$3,705-3,760** |

**Note:** Prices are approximate and may vary by vendor and region.

---

### 6.2 Component Sourcing

| Component | Suggested Vendor | Part Number / Link |
|-----------|-----------------|-------------------|
| Jetson AGX Orin 64GB | NVIDIA, Arrow, Digi-Key | 945-13730-0050-000 |
| OAK-D Lite | Luxonis Shop | OAK-D-LITE-AF |
| HyperX QuadCast S | Amazon, Best Buy | HyperX QuadCast S |
| **Pololu G2 24v21** | **Pololu** | **[2995](https://www.pololu.com/product/2995)** |
| **Turnigy 4S LiPo (3×)** | **HobbyKing** | **Turnigy 5000mAh 4S 20-30C** |
| XT60 Parallel Harness | HobbyKing, Amazon | 3× XT60 to 1× XT60 |
| 12V Buck Converter (10A) | Amazon, eBay | XL4015 or similar (≥10A) |
| PAM8403 Amplifier | Amazon, AliExpress | Generic PAM8403 module |
| DeAgostini Kit | DeAgostini subscription | R2-D2 1:2 scale |

**For detailed sourcing and setup procedures, see:**
- [010_PROJECT_GOALS_AND_SETUP.md](010_PROJECT_GOALS_AND_SETUP.md) - Complete BOM
- [003_JETSON_FLASHING_AND_DISPLAY_SETUP.md](003_JETSON_FLASHING_AND_DISPLAY_SETUP.md) - Jetson setup

---

## 7. System Connection Diagram

### 7.1 Complete Hardware Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    R2D2 HARDWARE ARCHITECTURE                   │
│                      (All Physical Connections)                 │
└─────────────────────────────────────────────────────────────────┘

                    NVIDIA Jetson AGX Orin 64GB
                    ┌─────────────────────────┐
                    │  12-core ARM CPU        │
                    │  504-core GPU           │
                    │  64GB RAM               │
                    └─────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
    [USB 3.0]           [USB Type-A]         [12V Power]
        │                    │                    │
        ↓                    ↓                    ↓
  OAK-D Lite          HyperX QuadCast      DC-DC Buck (14.8V→12V)
  Camera              S Microphone                │
  (30 FPS RGB)        (48kHz Audio)               ↓
                                            3× Turnigy 4S LiPo
                                            (14.8V, 5000mAh each)
                                            Parallel = 15Ah total
                                                  │
        ┌─────────────────────────────────────────┤
        │                                         │
   [J511 Pin 9]                              [14.8V Direct]
   I2S Audio                                      │
        │                                         ↓
        ↓                                   Motor Drivers
   PAM8403 Amp ────→ 8Ω Speaker            (Pololu G2 24v21)
                                                  │
                                                  ↓
        ┌─────────────────────────────────────────┤
        │                                         │
   [40-Pin GPIO]                            DC Motors (3×)
        │                                   - Left Wheel
        ├─ Pin 22 (GPIO17) ──→ White LED   - Right Wheel
        ├─ Pin 32 (GPIO32) ──→ Shutdown    - Dome (Head)
        ├─ Pin 1/17 (3.3V) ──→ LED Power
        └─ Pin 6 (GND)     ──→ LED Ground

   [J42 Header]
        │
        └─ Pin 4 + Pin 1 ──→ Boot/Wake Button

   [Future Phase 3 - GPIO]
        └─ Pins TBD        ──→ Motor Control Signals (💡 Proposed)
```

---

### 7.2 Power Distribution Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                    POWER DISTRIBUTION TREE                     │
└────────────────────────────────────────────────────────────────┘

                3× Turnigy 4S LiPo Batteries
                (14.8V nominal, 5000mAh each)
                Parallel Configuration (3P)
                Combined: 14.8V, 15Ah, 222Wh
                XT60 Parallel Harness
                        │
        ────────────────┼────────────────────────
        │               │                       │
        │               │                       │
   [14.8V Rail]    [12V Buck]              [5V Buck]
   High Current    (10A rated)             (3A rated)
        │               │                       │
        │               ↓                       ↓
        │        Jetson AGX Orin         Peripherals (Phase 3-4)
        │        J507 Barrel Jack        PAM8403 Amp
        │        (24-100W, 2-8A)         WS2812B LEDs (Phase 4)
        ↓
   Motor Drivers (Pololu G2 24v21)
   Each: 6.5-40V input, 21A continuous
        │
        ├─→ Driver #1 → Left Wheel Motor (21A peak)
        ├─→ Driver #2 → Right Wheel Motor (21A peak)
        └─→ Driver #3 → Dome Motor (21A peak, optional)

   Total Current Draw:
   - Typical: ~5A @ 14.8V (~75W)
   - Peak:    ~13A @ 14.8V (~190W)
   
   Battery C-Rating Required: 13A / 5Ah = 2.6C minimum
```

---

## 8. Phase Integration Roadmap

### 8.1 Hardware Integration Status by Phase

| Phase | Subsystem | Components | Status | Integration Date |
|-------|-----------|------------|--------|------------------|
| **Phase 1** | Core Perception | Jetson + OAK-D + LED + Power + Chassis | ✅ Complete | Nov-Dec 2025 |
| **Phase 2** | Speech & Audio | HyperX Mic + PAM8403 + Speaker | ✅ Complete | Dec 2025 |
| **Phase 3** | Navigation | Motors + Drivers + Servos + Encoders | ⏳ Planned | Q1-Q2 2026 |
| **Phase 4** | Personality | RGB LEDs + Advanced Expression | ⏳ Planned | Q3 2026+ |

### 8.2 Phase 3 Hardware Tasks (Future)

**Pending Integration:**
1. Wire wheel motors to MC33926 drivers (left + right)
2. Wire dome motor to motor driver
3. Connect motor encoder signals to GPIO (6 pins total)
4. Install and wire 5V DC-DC converter
5. Wire dome motor encoder signals to GPIO (for position feedback)
6. **[If sourced]** Wire servo control signals to GPIO PWM pins
7. Implement motor control ROS 2 nodes (PWM + odometry)
8. Test motor power draw and adjust battery/converter capacity if needed

### 8.3 Phase 4 Hardware Tasks (Future)

**Pending Integration:**
1. Install WS2812B RGB LED strip in chassis
2. Wire LED strip to 5V rail and GPIO data pin
3. Implement LED control library and ROS 2 node
4. Design LED animation patterns for status/personality

---

## 9. Jetson AGX Orin Connection Guide

### 9.1 Jetson AGX Orin Developer Kit Layout

**Reference:** [NVIDIA Jetson AGX Orin Developer Kit User Guide - Hardware Layout](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html)

![Jetson AGX Orin Top View](https://developer.nvidia.com/sites/default/files/akamai/embedded/images/jetsonAGXOrin/JetsonAGX_Orin_top_view.png)
*Image source: NVIDIA Developer Documentation (Licensed for reference)*

**Key Connectors for R2D2 Project:**

| Connector | Label | Function | R2D2 Usage |
|-----------|-------|----------|------------|
| **J507** | DC Barrel Jack | 9V-20V DC power input (5.5mm×2.1mm, center positive) | ✅ **Primary power** from 12V buck converter |
| **J508** | DC Power Header | 2-pin power input (VIN + GND) | Alternative to J507 barrel jack |
| **J511** | Audio Header | I2S audio output (includes HPO_L on Pin 9) | ✅ PAM8403 amplifier connection |
| **J42** | Automation Header | Power control (Pin 4 = POWER, Pin 1 = GND) | ✅ Boot/wake button |
| **J12** | 40-Pin GPIO Header | General purpose I/O (3.3V logic) | ✅ LED, buttons, motors (Phase 3) |
| **J30** | USB 3.0 Type-A | USB 3.0 host ports | ✅ OAK-D camera, HyperX mic |
| **J40** | USB-C Device | USB device mode (also 5V/3A power input) | USB networking (192.168.x.1) |

### 9.2 Power Connection Strategy for R2D2

#### Recommended Configuration: Barrel Jack (J507)

**Complete Power Chain:**
```
3× Turnigy 4S LiPo (14.8V, parallel)
    ↓ (XT60 parallel harness)
Main XT60 Connector
    ↓
Power Distribution Board
    ├─→ [14.8V Rail] → Motor Drivers (direct)
    ├─→ [12V Buck Converter] → Jetson J507 Barrel Jack
    └─→ [5V Buck Converter] → Servos + Peripherals
```

**Physical Connection to Jetson:**
1. **12V Buck Converter Output:** Solder barrel jack connector (5.5mm × 2.1mm, center positive)
2. **Connection:** Plug barrel jack into Jetson J507
3. **Polarity:** Center pin = +12V, Outer barrel = GND
4. **Voltage Check:** Verify 12V ±5% with multimeter before connecting
5. **Current Rating:** Ensure wiring supports ≥10A (use 18 AWG or thicker)

**Alternative: DC Power Header (J508)**

If barrel jack unavailable, use J508 header:
- **Pin 1 (VIN):** Connect to +12V from buck converter
- **Pin 2 (GND):** Connect to GND from buck converter
- **Wire Gauge:** 18 AWG minimum (for 10A current)
- **Connector:** 2-pin JST or direct solder to header

**Safety Considerations:**
- **Voltage Range:** Jetson accepts 9V-20V on J507/J508
  - Too low (<9V): Undervoltage shutdown
  - Too high (>20V): Permanent damage risk
- **Recommended:** 12V ±5% (11.4V - 12.6V safe range)
- **Fuse Protection:** Add 10A fuse between battery and Jetson converter
- **Reverse Polarity:** Buck converter should have reverse polarity protection

### 9.3 GPIO Header Pinout (J12) - 40-Pin Expansion

![Jetson AGX Orin GPIO Pinout](https://developer.nvidia.com/sites/default/files/akamai/embedded/images/jetsonAGXOrin/Jetson_AGX_Orin_GPIO_Header_Pinout.png)
*Image source: NVIDIA Developer Documentation (Licensed for reference)*

**Current Pin Assignments (Phase 1-2):**

| Physical Pin | GPIO Name | Function | R2D2 Connection |
|--------------|-----------|----------|-----------------|
| 1 | 3.3V | Power | White LED Panel (+) |
| 6 | GND | Ground | White LED Panel (-) |
| 22 | GPIO17 (PQ.05) | Output | White LED Control |
| 32 | GPIO12 (PN.01) | Input | Shutdown Button |

**Reserved Pins (Phase 3 - Motors):**

| Physical Pin | GPIO Name | Function | Proposed R2D2 Use |
|--------------|-----------|----------|-------------------|
| 11 | GPIO17 (PH.00) | PWM | Available for future use |
| 13 | GPIO27 (PN.00) | PWM | Available for future use |
| 15 | GPIO22 (PR.04) | PWM | Dome Motor PWM |
| 29 | GPIO05 (PAC.06) | Output | Dome Motor DIR |
| 31 | GPIO06 (PAA.00) | Output | Dome Motor EN |
| 33 | GPIO13 (PN.01) | PWM | Left Wheel PWM |
| 35 | GPIO19 (PQ.05) | Output | Left Wheel DIR |
| 37 | GPIO26 (PS.04) | Output | Left Wheel EN |
| 36 | GPIO16 (PQ.06) | Output | Right Wheel DIR |
| 38 | GPIO20 (PQ.07) | Output | Right Wheel EN |

**Note:** Pin assignments subject to change during Phase 3 integration. PWM-capable pins preferred for motor speed control.

### 9.4 Motor Driver Control Interface

**Pololu G2 24v21 to Jetson Wiring (per driver):**

| G2 Pin | Function | Jetson Connection | Wire Notes |
|--------|----------|-------------------|------------|
| **VIN** | Power input | 14.8V from battery (direct) | Heavy gauge (14-16 AWG) |
| **GND** | Power ground | Battery GND | Heavy gauge (14-16 AWG) |
| **OUTA** | Motor output A | Motor terminal + | Motor wire |
| **OUTB** | Motor output B | Motor terminal - | Motor wire |
| **DIR** | Direction control | GPIO pin (3.3V output) | Signal wire (22-26 AWG) |
| **PWM** | Speed control | GPIO PWM pin (3.3V) | Signal wire (22-26 AWG) |
| **SLP** | Sleep mode | Connect to 3.3V (always on) or GPIO | Optional control |
| **FLT** | Fault output | GPIO input (optional) | For fault monitoring |
| **CS** | Current sense | Analog input (optional) | For current monitoring |
| **VOUT** | Logic output | Not connected | Internal use |

**Example Wiring (Left Wheel Motor):**
```
Jetson GPIO Header (J12)
    ├─ Pin 33 (GPIO13) ──PWM──→ G2 Driver #1 [PWM pin]
    ├─ Pin 35 (GPIO19) ──DIR──→ G2 Driver #1 [DIR pin]
    ├─ Pin 37 (GPIO26) ──EN───→ G2 Driver #1 [SLP pin]
    └─ Pin 39 (GND) ───GND────→ G2 Driver #1 [GND pin] (signal ground)

Battery Power Rail
    ├─ (+14.8V) ──────────────→ G2 Driver #1 [VIN]
    └─ (GND) ─────────────────→ G2 Driver #1 [GND]

Left Wheel Motor
    ├─ Terminal + ────────────→ G2 Driver #1 [OUTA]
    └─ Terminal - ────────────→ G2 Driver #1 [OUTB]
```

**Important Notes:**
- **Common Ground:** Jetson GND and battery GND must be connected (shared ground plane)
- **Logic Level:** Pololu G2 accepts 3.3V logic directly (no level shifter needed)
- **Power Separation:** Motor power (VIN) is separate from logic power (Jetson GPIO)
- **Current Sense:** CS output provides ~20mV/A (can read with ADC for monitoring)

### 9.5 Battery to Jetson Power Budget

**Current Draw Analysis (3× Battery Parallel Configuration):**

| Load | Voltage | Current (Typical) | Current (Peak) | Power | Source |
|------|---------|-------------------|----------------|-------|--------|
| **Jetson AGX Orin** | 12V | 2-4A | 8.3A | 24-100W | 12V Buck (from battery) |
| **Left Wheel Motor** | 14.8V | 1-2A | 21A | 15-30W (typ), 310W (peak) | Battery direct (via G2 driver) |
| **Right Wheel Motor** | 14.8V | 1-2A | 21A | 15-30W (typ), 310W (peak) | Battery direct (via G2 driver) |
| **Dome Motor** | 14.8V | 0.5-1A | 10A | 7-15W (typ), 148W (peak) | Battery direct (via G2 driver) |
| **Total from Battery** | 14.8V | **~8A** | **~55A** | **~120W (typ), 815W (peak)** | — |

**Battery Discharge Analysis:**

With **3× Turnigy 5000mAh 4S batteries in parallel:**
- **Total Capacity:** 15,000mAh (15Ah) @ 14.8V
- **Total Energy:** 222Wh
- **Continuous Discharge (20C rating):** 15Ah × 20C = **300A** (far exceeds needs)
- **Typical Draw:** 8A (0.53C - very safe)
- **Peak Draw:** 55A (3.7C - within safe limits)

**Runtime Estimates:**
- **Phase 1-2 (stationary):** ~35W average → **6.3 hours**
- **Phase 3 (cruising):** ~75W average → **3.0 hours**
- **Phase 3 (active):** ~120W average → **1.85 hours**

**Conclusion:** 3× battery parallel configuration provides excellent capacity and safety margins.

---

## 10. Critical Hardware Notes

### 9.1 Known Issues & Limitations

| Issue | Severity | Workaround | Status |
|-------|----------|------------|--------|
| OAK-D must be direct USB connection (not hub) | High | Always connect directly to Jetson | ✅ Documented |
| GPIO pin conflicts (shutdown vs motors) | Medium | Reassign pins during Phase 3 | ⏳ Planning |
| Insufficient GPIO pins for all functions | Medium | Use I2C PWM controller for servos | 💡 Proposed |
| Battery discharge rate unknown | Medium | Measure C-rating before motor integration | ⏳ Pending |
| Speaker amplifier separate power needed | Low | External power supply for PAM8403 | ✅ Resolved |

### 9.2 Hardware Safety Considerations

**Electrical Safety:**
- LiPo battery requires proper charging (balance charger, fire-safe bag)
- Never discharge LiPo below 14.0V (3.5V per cell)
- Add fuses to power distribution (recommended: 10A main, 5A on 12V rail)
- Motor drivers have over-current protection (MC33926 auto-shutdown)

**Mechanical Safety:**
- Servo mounting must be secure (vibration can loosen screws)
- Motor stall current can damage drivers (implement current limiting)
- Ensure dome rotation range is unobstructed (camera rotates with dome)

**Thermal Management:**
- Jetson requires active cooling at high loads (fan or heatsink)
- DC-DC converters need heatsinking (especially 12V @ 8A)
- Monitor temps via `tegrastats` (Jetson) and thermal camera if available

---

## 10. Cross-References & Related Documentation

### 10.1 Hardware Setup Guides
- [003_JETSON_FLASHING_AND_DISPLAY_SETUP.md](003_JETSON_FLASHING_AND_DISPLAY_SETUP.md) - Jetson flashing, display setup, SDK Manager
- [020_POWER_BUTTON_FINAL_DOCUMENTATION.md](020_POWER_BUTTON_FINAL_DOCUMENTATION.md) - Power button wiring and testing
- [HARDWARE_WHITE_LED_WIRING.md](HARDWARE_WHITE_LED_WIRING.md) - LED panel wiring diagram and troubleshooting

### 10.2 Software Integration
- [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - Software architecture, ROS 2 nodes, data flow
- [007_SYSTEM_INTEGRATION_REFERENCE.md](007_SYSTEM_INTEGRATION_REFERENCE.md) - Complete system integration guide
- [100_PERSON_RECOGNITION_REFERENCE.md](100_PERSON_RECOGNITION_REFERENCE.md) - Face recognition setup
- [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture

### 10.3 Component-Specific Documentation
- **OAK-D Camera:** [102_CAMERA_SETUP_DOCUMENTATION.md](102_CAMERA_SETUP_DOCUMENTATION.md) (referenced from architecture docs)
- **Audio System:** [101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md](101_SPEAKER_AUDIO_SETUP_DOCUMENTATION.md) (referenced from architecture docs)
- **Web Dashboard:** [110_WEB_UI_REFERENCE.md](110_WEB_UI_REFERENCE.md) - Remote monitoring interface

### 10.4 Project Planning
- [010_PROJECT_GOALS_AND_SETUP.md](010_PROJECT_GOALS_AND_SETUP.md) - Complete BOM, project phases, success criteria
- [README.md](README.md) - Project overview, quick start, roadmap

---

## 11. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-20 | Severin Leuenberger | Initial hardware reference document created |

---

**Document Status:** ✅ Complete for Phase 1-2, 💡 Proposed designs for Phase 3-4  
**Last Updated:** December 20, 2025  
**Maintainer:** Severin Leuenberger

---

**End of Hardware Reference Document**

