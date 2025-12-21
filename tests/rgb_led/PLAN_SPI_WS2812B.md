# SPI-Based WS2812B Control Plan

**Goal:** Control WS2812B LED panel using Jetson's SPI hardware
**Timeline:** 2-3 iterations
**Reversibility:** Fully reversible (no permanent system changes)

---

## Architecture

### How WS2812B SPI Works

WS2812B LEDs need precise 800kHz timing:
- **Bit 0:** 0.4μs HIGH, 0.85μs LOW (total 1.25μs)
- **Bit 1:** 0.8μs HIGH, 0.45μs LOW (total 1.25μs)
- **Reset:** >50μs LOW

We use SPI at 6.4MHz to encode each WS2812B bit as 8 SPI bits:
- WS2812B bit 0 → SPI: 0b11000000 (short pulse)
- WS2812B bit 1 → SPI: 0b11110000 (long pulse)

---

## Iteration 1: Enable SPI and Verify (CURRENT)

### Step 1.1: Check Available SPI Devices

```bash
ls -l /dev/spidev*
```

**Expected:** May not exist yet (SPI not enabled)

### Step 1.2: Enable SPI via Jetson-IO

```bash
# Run Jetson-IO configuration tool
sudo /opt/nvidia/jetson-io/jetson-io.py
```

**Steps in menu:**
1. Select "Configure Jetson 40-pin header"
2. Find SPI options (SPI1 or SPI3)
3. Enable SPI
4. Save and reboot

**Alternative (if jetson-io doesn't work):**
```bash
# Manual device tree overlay (if needed)
sudo /opt/nvidia/jetson-io/config-by-pin.py -o dtb
```

### Step 1.3: Verify SPI After Reboot

```bash
ls -l /dev/spidev*
# Should show: /dev/spidev0.0 or /dev/spidev3.0
```

### Step 1.4: Install Python SPI Library

```bash
sudo pip3 install spidev
```

### Step 1.5: Test SPI Communication

```bash
python3 test_05_spi_basic.py
```

**Rollback if needed:**
```bash
sudo /opt/nvidia/jetson-io/jetson-io.py  # Disable SPI
sudo pip3 uninstall -y spidev
```

---

## Iteration 2: WS2812B SPI Driver (NEXT)

### Step 2.1: Create WS2812B Driver Class

**File:** `ws2812b_spi_driver.py`

**Key features:**
- Encode RGB values to SPI format
- Handle 16 LEDs in circular array
- Set individual or all LEDs
- Brightness control

### Step 2.2: Test Basic Colors

```bash
python3 test_06_spi_colors.py
```

**Tests:**
- All LEDs RED
- All LEDs GREEN
- All LEDs BLUE
- All LEDs WHITE
- All LEDs OFF

---

## Iteration 3: Status Integration (FINAL)

### Step 3.1: Status Color Mapping

**Map recognition states to LED colors:**
- RED state (recognized) → LED = RED
- BLUE state (lost) → LED = BLUE
- GREEN state (unknown) → LED = GREEN

### Step 3.2: Integrate with status_led_node

**Add SPI mode to existing node:**
- `led_type` = 'ws2812b_spi'
- Use SPI driver class
- Update on status changes

---

## Hardware Connections

**Current setup (verify):**
```
LED Panel:
├─ Red wire → Pin 4 (5V) ✅
├─ Black wire → Pin 6 (GND) ✅
└─ Blue wire → Will connect to SPI MOSI pin
```

**SPI Pin Options on Jetson AGX Orin:**
- **SPI1 MOSI:** Pin 19 (physical)
- **SPI3 MOSI:** Pin 21 (physical) - alternative

**Final wiring:**
```
Blue wire (data) → Pin 19 (SPI1 MOSI) or Pin 21 (SPI3 MOSI)
```

---

## Rollback Plan

**Complete removal (if SPI doesn't work):**

```bash
# 1. Disable SPI
sudo /opt/nvidia/jetson-io/jetson-io.py
# (Uncheck SPI in menu)

# 2. Remove Python library
sudo pip3 uninstall -y spidev

# 3. Remove test files
rm ~/dev/r2d2/tests/rgb_led/test_05_spi_basic.py
rm ~/dev/r2d2/tests/rgb_led/test_06_spi_colors.py
rm ~/dev/r2d2/tests/rgb_led/ws2812b_spi_driver.py

# 4. Reboot
sudo reboot
```

**Result:** System back to original state, no changes persist.

---

## Success Criteria

**Iteration 1:** SPI device appears at `/dev/spidev*`
**Iteration 2:** LEDs light up in RED/GREEN/BLUE
**Iteration 3:** Status system controls LED colors

---

**Status:** ITERATION 1 IN PROGRESS
**Next:** Enable SPI and verify device appears

