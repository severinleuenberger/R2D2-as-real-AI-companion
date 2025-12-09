# R2D2 Power Button - Final Working Setup

## Overview
R2D2 now has a **two-button power control system**:
- **Button 1**: Software control (low-power mode, shutdown)
- **Button 2**: Hardware control (wake/boot)

## Button 1: Software Control Button
**Location**: 40-pin header on Jetson AGX Orin

### Wiring
| Component | Pin |
|-----------|-----|
| Button Signal | Pin 32 (GPIO09) |
| Ground | Pin 39 (GND) |

### Functionality
- **Single Press**: Enter low-power mode (`nvpmodel -m 2`)
- **Double Press**: Graceful shutdown (`shutdown -h now`)

### How to Use
1. Press once → System enters low-power mode (reduced CPU/GPU frequency)
2. Press twice quickly (within 500ms) → System shuts down gracefully

## Button 2: Hardware Power Control Button
**Location**: J42 Automation Header on Jetson AGX Orin

### Wiring
| Component | Pin |
|-----------|-----|
| Button Signal | J42 Pin 2 (PWR_BTN) |
| Ground | J42 Pin 1 (GND) |

### Functionality
- **Single Press**: Wake from low-power mode OR boot from shutdown
- Works regardless of system state (OFF, low-power, or ON)

### How to Use
1. When system is in low-power mode → Press to wake back to normal mode
2. When system is shutdown → Press to boot
3. When system is running → Ignored (can be configured to shutdown if desired)

## Complete Usage Workflow

### Scenario 1: Enter Low-Power and Wake
```
System Running
    ↓
Press Button 1 (single) → Low-Power Mode
    ↓
Press Button 2 (single) → Wake back to Normal
```

### Scenario 2: Shutdown and Boot
```
System Running
    ↓
Press Button 1 (double) → Shutdown
    ↓
System OFF
    ↓
Press Button 2 (single) → Boot
```

### Scenario 3: Quick Restart
```
System Running
    ↓
Press Button 1 (double) → Shutdown
    ↓
Press Button 2 (single) → Boot back up
```

## Technical Details

### Software Implementation
- **GPIO Pin**: 32 (GPIO09 on Jetson AGX Orin)
- **Debounce Time**: 100ms (prevents electrical noise)
- **Double-Press Window**: 500ms (time between presses to register as double)
- **Long-Press Threshold**: 2.0 seconds (prevents accidental triggers)

### Service
- **Service Name**: `r2d2-powerbutton.service`
- **Status**: `systemctl status r2d2-powerbutton.service`
- **Start/Stop**: `sudo systemctl start/stop r2d2-powerbutton.service`
- **Logs**: `journalctl -u r2d2-powerbutton.service -f`

### Hardware Considerations
- **Button 1** uses internal GPIO pull-up resistor (Pin 32 is GPIO09)
- **Button 2** uses hardware PWR_BTN signal (works at power level)
- Both buttons are momentary push buttons (normally open)
- Button 1 connected to 3.3V GPIO logic
- Button 2 connected to Jetson power management hardware

## Files
- **Code**: `/usr/local/bin/r2d2_power_button.py`
- **Service**: `/etc/systemd/system/r2d2-powerbutton.service`
- **Logs**: `/var/log/r2d2_power_button.log`
- **Source**: `/home/severin/dev/r2d2/r2d2_power_button.py`

## Troubleshooting

### Button 1 not responding
1. Check GPIO Pin 32 connection to Pin 39
2. Verify service is running: `sudo systemctl status r2d2-powerbutton.service`
3. Check logs: `journalctl -u r2d2-powerbutton.service -n 50`

### Button 2 not working (hardware power)
1. Check J42 Pin 1 (GND) and Pin 2 (PWR_BTN) connections
2. Verify button is making electrical contact (use multimeter)
3. Try pressing and holding for 1-2 seconds

### System not entering low-power mode
1. Verify `nvpmodel` is installed: `which nvpmodel`
2. Check sudo permissions for power button service
3. Look at logs for errors

## Tested Configuration
- **Device**: NVIDIA Jetson AGX Orin Developer Kit
- **OS**: Ubuntu 22.04, JetPack 6.x
- **Python**: 3.10+
- **Library**: Jetson.GPIO
- **Date Tested**: December 9, 2025
- **Status**: ✅ WORKING

## Notes
- Button 1 handles all software power management (low-power/shutdown)
- Button 2 is dedicated to hardware power control (wake/boot)
- This separation ensures reliable operation even if service crashes
- Both buttons can be pressed independently without interference
