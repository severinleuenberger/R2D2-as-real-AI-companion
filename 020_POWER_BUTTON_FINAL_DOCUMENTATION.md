# R2D2 Power Button System - Final Implementation

## Overview

Simple, reliable power button control for R2D2 robot using two buttons on the Jetson AGX Orin.

**Status: ✅ TESTED AND OPERATIONAL**

## Hardware Setup

### Button 1: Shutdown Control
- **Function**: Press to shutdown the system
- **Location**: 40-pin expansion header
- **Wiring**:
  - Pin 32 (GPIO09) ← Button signal
  - Pin 39 (GND) ← Ground return
- **Status**: ✅ WORKING - Tested and verified

### Button 2: Boot/Wake Control
- **Function**: Short pins to wake from low-power or boot from shutdown
- **Location**: J42 Automation Header
- **Wiring**:
  - Pin 4 (POWER) ← Button signal
  - Pin 1 (GND) ← Ground return
- **Status**: Ready for testing

**Note**: J42 Pin 4 (POWER) is the correct pin for software-controlled boot/wake. Pin 2 (PWR_BTN) is hardware-only.

## Software Implementation

### File Locations
- **Source**: `/home/severin/dev/r2d2/r2d2_power_button_simple.py`
- **Deployed**: `/usr/local/bin/r2d2_power_button.py`
- **Service**: `/etc/systemd/system/r2d2-powerbutton.service`

### Service Configuration
```bash
# Check status
sudo systemctl status r2d2-powerbutton.service

# View logs
journalctl -u r2d2-powerbutton.service -f

# Restart service
sudo systemctl restart r2d2-powerbutton.service
```

### Logging
- **File**: `/var/log/r2d2_power_button.log`
- **Format**: Timestamp [LEVEL] Message
- **Automatic rotation**: Handled by systemd

## Button Functions

### Button 1 (Pin 32)
```
Press and release → Play R2-D2 sound → Shutdown (shutdown -h now)
- Detection: Any press triggers shutdown sequence
- Audio: Plays MP3 file before shutdown (system default volume)
- Debounce: 100ms
- Response time: Immediate (audio plays, then shutdown)
- Log: "Button pressed" → "Playing shutdown sound" → "Initiating shutdown"
- Audio file: /home/severin/Voicy_R2-D2 - 3.mp3
- Audio player: ffplay (with -nodisp -autoexit flags)
- Timeout: 30 seconds maximum playback time
- Error handling: Shutdown proceeds even if audio fails
```

### Button 2 (J42 Pin 4 + Pin 1)
```
Short the pins → Wake/Boot system
- Detection: Hardware-level power signal
- No debounce needed (hardware-controlled)
- Response: System wakes from low-power or boots from shutdown
```

## Testing Results

### Test 1: Button 1 Shutdown ✅ PASS
- **Time**: 2025-12-09 07:30:40
- **Test**: Single button press
- **Result**: 
  ```
  07:30:40 Button pressed
  07:30:41 Button released (duration: 0.32s)
  07:30:41 Initiating shutdown...
  07:30:41 ACTION: Shutting down system...
  ```
- **Verification**: System shutdown gracefully, service restarted on boot

### Test 2: Button 2 Boot/Wake ⏳ READY
- **Status**: Hardware wired, code ready
- **Expected**: System wakes when pins shorted
- **Test**: Short J42 Pin 4 + Pin 1 after shutdown

## Installation

The power button system is pre-installed and auto-starting:

1. **Service enabled**: `systemctl is-enabled r2d2-powerbutton.service` (should show "enabled")
2. **Service running**: `systemctl is-active r2d2-powerbutton.service` (should show "active")
3. **Auto-restart**: Service restarts automatically on failure

## Code Architecture

### PowerButtonHandler Class
Monitors GPIO Pin 32 for button presses:
- Debounces electrical noise (100ms threshold)
- Detects falling edge (press) and rising edge (release)
- Executes shutdown sequence on any valid press

### Shutdown Sequence
```python
def execute_shutdown():
    play_shutdown_sound()  # Play R2-D2 MP3
    shutdown -h now        # Graceful shutdown
```

### Audio Playback Function
- **Function**: `play_shutdown_sound()`
- **File**: `/home/severin/Voicy_R2-D2 - 3.mp3`
- **Player**: `ffplay -nodisp -autoexit -loglevel quiet`
- **Volume**: System default (no volume override)
- **Timeout**: 30 seconds maximum
- **Error handling**: Always proceeds with shutdown even if audio fails

### Main Loop
```python
while True:
    current_state = GPIO.input(BUTTON_GPIO_PIN)
    # Debounce logic
    if state_stable and state_changed:
        if button_pressed:
            execute_shutdown()  # Plays sound, then shuts down
    time.sleep(POLLING_INTERVAL)  # 20ms
```

## Troubleshooting

### Button not responding
1. Check GPIO wiring (Pin 32 + Pin 39)
2. Verify service is running: `sudo systemctl status r2d2-powerbutton.service`
3. Check logs: `tail -50 /var/log/r2d2_power_button.log`
4. Restart service: `sudo systemctl restart r2d2-powerbutton.service`
5. **Verify Pin 32 is configured as GPIO:** Check that no device tree overlay is overriding Pin 32
   ```bash
   # Check if Jetson-IO overlay is active
   cat /boot/extlinux/extlinux.conf | grep -i overlay
   # Should NOT show jetson-io-hdr40-user-custom.dtbo if using default config
   ```

### Pin Configuration Issue (December 2025 - RESOLVED)
**Problem:** Power button was not working - Pin 32 was configured as `dmic3_clk` instead of GPIO09.

**Root Cause:** 
- Jetson-IO tool had created a custom device tree overlay (`jetson-io-hdr40-user-custom.dtbo`)
- This overlay configured Pin 32 as `dmic3_clk` (digital microphone clock) instead of GPIO09
- The overlay was loaded at boot via `/boot/extlinux/extlinux.conf` with `DEFAULT JetsonIO`

**Solution Applied:**
1. Changed boot configuration to use default pins:
   ```bash
   # Changed DEFAULT from "JetsonIO" to "primary" in /boot/extlinux/extlinux.conf
   sudo sed -i 's/^DEFAULT JetsonIO/DEFAULT primary/' /boot/extlinux/extlinux.conf
   ```
2. Removed JetsonIO section from extlinux.conf:
   ```bash
   sudo sed -i '/^LABEL JetsonIO/,/^	OVERLAYS/d' /boot/extlinux/extlinux.conf
   ```
3. Deleted custom overlay file:
   ```bash
   sudo rm /boot/jetson-io-hdr40-user-custom.dtbo
   ```

**Result:** 
- ✅ Pin 32 is now back to default GPIO09 configuration
- ✅ Power button works correctly
- ✅ System uses original Jetson AGX Orin 40-pin header configuration

**Important Notes:**
- The system is now using the **original/default Jetson pin configuration**
- Pin 32 (GPIO09) is available for GPIO use (power button)
- Pin 27 (GPIO27) is available for GPIO use (LED control)
- Pin 22 (GPIO22) is available for GPIO use (LED control)
- Pin 17 (GPIO17) is available for GPIO use (LED control)
- **No device tree overlays are active** - all pins use their default functions
- If you need to reconfigure pins in the future, use Jetson-IO tool carefully and verify pin assignments

### Service not starting
1. Check service file: `cat /etc/systemd/system/r2d2-powerbutton.service`
2. Check Python syntax: `python3 -m py_compile /usr/local/bin/r2d2_power_button.py`
3. Reload systemd: `sudo systemctl daemon-reload`

### Shutdown not working
1. Verify button press in logs: `journalctl -u r2d2-powerbutton.service -n 20`
2. Test GPIO manually:
   ```bash
   python3 << 'EOF'
   import Jetson.GPIO as GPIO
   GPIO.setmode(GPIO.BOARD)
   GPIO.setup(32, GPIO.IN)
   print(GPIO.input(32))  # Should be HIGH
   EOF
   ```
3. Test shutdown command: `sudo shutdown -h now` (will shutdown!)

## Files

```
r2d2_power_button_simple.py       - Main handler code (~150 lines)
r2d2-powerbutton.service           - Systemd service definition
POWER_BUTTON_FINAL_DOCUMENTATION.md - This file
```

## Key Specifications

| Parameter | Value |
|-----------|-------|
| GPIO Pin | 32 (40-pin header) |
| Debounce | 100ms |
| Polling Interval | 20ms |
| Log File | /var/log/r2d2_power_button.log |
| Service | r2d2-powerbutton (auto-start, auto-restart) |
| Action on Press | Play R2-D2 sound → Graceful shutdown |
| Audio File | /home/severin/Voicy_R2-D2 - 3.mp3 |
| Audio Player | ffplay (system default volume) |
| Audio Timeout | 30 seconds |
| J42 Pin | 4 (POWER) for boot/wake |

## Next Steps

1. ✅ Button 1 (shutdown): Tested and working
2. ⏳ Button 2 (boot/wake): Ready to test - short J42 Pin 4 + Pin 1
3. Optional: Add low-power mode (nvpmodel) or other power states

## Support

For issues or modifications:
- Check logs: `journalctl -u r2d2-powerbutton.service -f`
- View file logs: `tail -f /var/log/r2d2_power_button.log`
- Service status: `sudo systemctl status r2d2-powerbutton.service`

---

## 40-Pin Header Configuration Status

**Current Configuration:** ✅ **Original/Default Jetson AGX Orin Configuration**

The system has been reverted to the original Jetson pin configuration. No custom device tree overlays are active.

**Pin Assignments (Current):**
- Pin 32: GPIO09 (Power Button - ✅ Working)
- Pin 17: GPIO17 (RED LED - Available)
- Pin 27: GPIO27 (GREEN LED - Available)
- Pin 22: GPIO22 (BLUE LED - Available)

**What This Means:**
- All GPIO pins are available for their default functions
- No conflicts with audio (DMIC) or I2C functions
- System is in a clean, predictable state
- Future pin configurations should be done carefully to avoid conflicts

**If You Need to Reconfigure Pins:**
1. Use Jetson-IO tool: `sudo /opt/nvidia/jetson-io/jetson-io.py`
2. **Verify pin assignments** before saving
3. Test GPIO functionality after any changes
4. Document any custom overlays created

---

**Last Updated**: 2025-12-16
**Status**: Production Ready (Button 1 verified and working with audio playback, Button 2 ready)
**Configuration**: Original Jetson default (no custom overlays)
**Features**: Shutdown with R2-D2 audio feedback
