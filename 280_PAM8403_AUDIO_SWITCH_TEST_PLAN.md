# PAM8403 Audio Switch Test Plan

> **Purpose**: Testing procedure for physical GPIO audio switch between PAM8403 onboard speaker and Bluetooth output  
> **Status**: Hardware disconnected - use this plan when PAM8403 is repaired/reconnected  
> **Last Updated**: January 2, 2026

---

## Overview

This document provides isolated testing procedures for the GPIO-based audio output switch that selects between:
- **PAM8403 Onboard Speaker** (ALSA via I2S)
- **Bluetooth Audio** (FreeBuds 4i via A2DP)

**Design Philosophy:** SIMPLE, STABLE, RELIABLE, NON-RISKY
- Audio outputs are **EXCLUSIVE** (never both at once)
- Physical switch provides clear, deterministic selection
- No parallel audio complexity
- Easy debugging (check switch position = know audio output)

---

## Hardware Setup

### Toggle Switch Specification

| Component | Specification |
|-----------|--------------|
| **Switch Type** | SPST (Single Pole Single Throw) toggle switch |
| **GPIO Pin** | GPIO 17 (Physical Pin 11 on 40-pin header) |
| **Ground Pin** | Physical Pin 9 (GND) |
| **Pull-up Resistor** | 10kΩ to 3.3V (Physical Pin 1) |
| **Switch Position UP** | Open circuit → GPIO HIGH → Bluetooth |
| **Switch Position DOWN** | Closed to GND → GPIO LOW → PAM8403 |

### Wiring Diagram

```
40-Pin Header Connections
┌────────────────────────────────────────┐
│  Pin 1 (3.3V) ──┬── 10kΩ Resistor ──┬──┐
│                 │                    │  │
│  Pin 11 (GPIO 17) ───────────────────┘  │
│                                       │  │
│  Pin 9 (GND) ──── Switch ─────────────┘
└────────────────────────────────────────┘

Switch Logic:
- UP (Open):   No connection to GND → Pull-up keeps GPIO HIGH → Bluetooth
- DOWN (Closed): Connected to GND → GPIO LOW → PAM8403
```

### Parts List

- 1× SPST toggle switch (any standard size)
- 1× 10kΩ resistor (1/4W or 1/8W)
- 3× Female-to-female jumper wires
- Optional: Small project box or mounting bracket

---

## Phase 1: Hardware Installation

### Prerequisites

- PAM8403 speaker hardware repaired and reconnected
- All soldering complete and verified
- R2D2 system powered OFF

### Installation Steps

1. **Power off R2D2:**
   ```bash
   sudo shutdown -h now
   # Wait for complete shutdown before proceeding
   ```

2. **Install pull-up resistor:**
   - Connect one leg to Pin 1 (3.3V)
   - Connect other leg to Pin 11 (GPIO 17)

3. **Connect switch:**
   - Terminal 1: Connect to Pin 11 (GPIO 17) [shares with resistor]
   - Terminal 2: Connect to Pin 9 (GND)

4. **Verify connections:**
   - Check resistor is between 3.3V and GPIO 17
   - Check switch is between GPIO 17 and GND
   - Ensure no shorts to adjacent pins

5. **Power on R2D2:**
   ```bash
   # Use boot button (J42) to power on
   ```

---

## Phase 2: GPIO Switch Testing (Isolated)

**CRITICAL:** Test the switch hardware BEFORE integrating with audio system!

### Step 2.1: Run GPIO Switch Test Script

```bash
cd ~/dev/r2d2/scripts
python3 test_gpio_switch.py
```

**Expected output:**
```
==================================================
GPIO SWITCH TEST
==================================================
Pin: GPIO 17 (Physical Pin 11)
Expected behavior:
  - Switch UP (open):   Should read HIGH (1)
  - Switch DOWN (closed): Should read LOW (0)
==================================================

Flip the switch to test. Press Ctrl+C to exit.
```

### Step 2.2: Test Switch Positions

1. **Flip switch to UP position:**
   ```
   Expected: [HH:MM:SS] Switch: UP (HIGH) → Would select: BLUETOOTH
   ```

2. **Flip switch to DOWN position:**
   ```
   Expected: [HH:MM:SS] Switch: DOWN (LOW) → Would select: PAM8403
   ```

3. **Flip rapidly several times:**
   - Verify each transition is detected
   - No missed states
   - Clean HIGH/LOW readings (no flickering)

4. **Leave in middle position (if applicable):**
   - Some switches have undefined middle state
   - Should read consistently (either HIGH or LOW)

### Step 2.3: Verification Checklist

- [ ] Switch UP → GPIO reads HIGH consistently
- [ ] Switch DOWN → GPIO reads LOW consistently  
- [ ] State changes detected within 1 second
- [ ] No spurious state changes when not touching switch
- [ ] Script runs without errors or warnings
- [ ] Clean GPIO cleanup on Ctrl+C

**If any checks fail:** Review wiring, check resistor value, verify GPIO pin number.

---

## Phase 3: PAM8403 Audio Testing

### Step 3.1: Verify PAM8403 ALSA Device

```bash
# List all audio sinks
pactl list sinks short

# Expected output should include:
# alsa_output.platform-sound.stereo-fallback  module-alsa-card.c ...
```

### Step 3.2: Test PAM8403 Direct Playback

```bash
# Set PAM8403 as default sink
pactl set-default-sink alsa_output.platform-sound.stereo-fallback

# Test with system sound
paplay /usr/share/sounds/alsa/Front_Center.wav
# Should hear from PAM8403 speaker

# Test with R2D2 beep
ffplay -autoexit -nodisp ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
# Should hear from PAM8403 speaker
```

### Step 3.3: Verification Checklist

- [ ] PAM8403 sink appears in `pactl list sinks short`
- [ ] System sound plays clearly through PAM8403
- [ ] R2D2 beep plays clearly through PAM8403
- [ ] No distortion or crackling
- [ ] Volume at reasonable level
- [ ] No PulseAudio errors in logs

---

## Phase 4: Audio Switch Service Testing

### Step 4.1: Enable Audio Switch Service

```bash
# Copy service file (if not already present)
sudo cp ~/dev/r2d2/r2d2-audio-switch.service /etc/systemd/system/

# Reload and enable
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-switch.service
sudo systemctl start r2d2-audio-switch.service
```

### Step 4.2: Verify Service Status

```bash
# Check service is running
systemctl status r2d2-audio-switch.service

# Expected: active (running)

# Check logs
journalctl -u r2d2-audio-switch.service -f
```

### Step 4.3: Test Switch Functionality

**Test 1: Bluetooth → PAM8403**
1. Flip switch to UP position (Bluetooth)
2. Check logs: Should show "Switch changed → BLUETOOTH"
3. Play test sound: `paplay /usr/share/sounds/alsa/Front_Center.wav`
4. Verify audio from FreeBuds 4i
5. Flip switch to DOWN position (PAM8403)
6. Check logs: Should show "Switch changed → PAM8403"
7. Play test sound again
8. Verify audio from PAM8403 speaker

**Test 2: PAM8403 → Bluetooth**
1. Start with switch DOWN (PAM8403)
2. Play continuous audio: `speaker-test -t wav -c 2`
3. Flip switch to UP (Bluetooth)
4. Audio should switch to FreeBuds 4i within 0.5 seconds
5. Stop test: Ctrl+C

**Test 3: Rapid Switching**
1. Play audio file in loop
2. Flip switch UP and DOWN several times
3. Verify smooth transitions
4. No audio dropouts or crashes

### Step 4.4: Verification Checklist

- [ ] Service starts without errors
- [ ] Switch changes detected in logs
- [ ] Audio output switches correctly
- [ ] Bluetooth position routes to FreeBuds 4i
- [ ] PAM8403 position routes to onboard speaker
- [ ] No audio glitches during switching
- [ ] Service survives rapid switch changes

---

## Phase 5: Integration Testing with R2D2 Services

### Step 5.1: Test with Gesture Feedback Beeps

```bash
# Monitor gesture intent service
journalctl -u r2d2-gesture-intent.service -f

# Manually trigger beep (simulate gesture acknowledgment)
ffplay -autoexit -nodisp ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 16.mp3
```

**Test both switch positions:**
- UP → Beep should play through Bluetooth
- DOWN → Beep should play through PAM8403

### Step 5.2: Test with Speech TTS Output

```bash
# Start minimal monitor
cd ~/dev/r2d2
python3 tools/minimal_monitor.py

# Test both switch positions:
```

**Switch DOWN (PAM8403):**
1. Trigger index finger gesture
2. Speak to R2D2
3. Verify TTS response plays through PAM8403
4. Stop conversation with fist gesture

**Switch UP (Bluetooth):**
1. Trigger index finger gesture
2. Speak to R2D2
3. Verify TTS response plays through FreeBuds 4i
4. Stop conversation with fist gesture

### Step 5.3: Test with All Audio Sources

```bash
# Test all audio sources with switch in BOTH positions:
```

**Audio Sources to Test:**
1. Recognition beep (audio_notification_node)
2. Loss beep (audio_notification_node)
3. Gesture acknowledgment beep (gesture_intent_node)
4. Session start beep (gesture_intent_node)
5. Session stop beep (gesture_intent_node)
6. Speech TTS output (speech_node)

**For each position:**
- [ ] All beeps audible and clear
- [ ] TTS speech intelligible
- [ ] No dropouts or glitches
- [ ] Consistent volume levels

---

## Phase 6: Long-Term Stability Testing

### Step 6.1: Endurance Test

```bash
# Run for 1 hour:
# - Switch positions every 5 minutes
# - Trigger conversations in each position
# - Monitor logs for errors
journalctl -u r2d2-audio-switch.service -f
```

### Step 6.2: Service Restart Test

```bash
# Test service recovery
sudo systemctl restart r2d2-audio-switch.service
sudo systemctl restart r2d2-speech-node.service
sudo systemctl restart r2d2-gesture-intent.service

# Verify switch still works after restarts
```

### Step 6.3: System Reboot Test

```bash
# Full system reboot
sudo reboot

# After boot:
# - Verify audio switch service auto-starts
# - Test switch positions work correctly
# - Verify default position is sensible
```

---

## Troubleshooting

### Issue: Switch Not Detected

**Symptoms:**
- No log messages when flipping switch
- Always selects same output regardless of position

**Checks:**
```bash
# Test GPIO manually
python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP); print(f'GPIO 17: {GPIO.input(17)}'); GPIO.cleanup()"

# Check service logs
journalctl -u r2d2-audio-switch.service --no-pager | tail -30
```

**Possible Causes:**
- Wiring issue (check connections)
- Wrong GPIO pin in script (verify pin 17)
- Service not running
- GPIO permissions issue

### Issue: Audio Doesn't Switch

**Symptoms:**
- Switch detected in logs
- But audio continues on wrong output

**Checks:**
```bash
# Check current default sink
pactl get-default-sink

# Check if sink exists
pactl list sinks short | grep -E "bluez|alsa"

# Try manual switch
pactl set-default-sink bluez_sink.28_54_71_BB_C6_53.a2dp_sink
paplay /usr/share/sounds/alsa/Front_Center.wav
```

**Possible Causes:**
- Sink name incorrect in script
- PulseAudio not reacting to default sink change
- Application using hardcoded sink

### Issue: PAM8403 No Audio

**Symptoms:**
- Switch to PAM8403 position
- No sound from speaker
- No PulseAudio errors

**Checks:**
```bash
# Check ALSA device
aplay -l | grep -i ape

# Test ALSA direct
speaker-test -D hw:APE,0 -c 2 -t wav

# Check PulseAudio sink
pactl list sinks | grep -A 10 "alsa_output"
```

**Refer to:** [`262_BLUETOOTH_AUDIO_OPEN_POINTS.md`](262_BLUETOOTH_AUDIO_OPEN_POINTS.md) Section 2 (PAM8403 Hardware Diagnosis)

### Issue: Bluetooth Disconnects When Switching

**Symptoms:**
- Switching to PAM8403 disconnects Bluetooth
- Need to manually reconnect

**Solution:**
- This is normal PulseAudio behavior
- Bluetooth stays paired, just audio suspended
- Will auto-resume when switching back

---

## Success Criteria

### Phase 2 Complete When:

- [ ] GPIO switch test script works correctly
- [ ] Switch positions read accurately (HIGH/LOW)
- [ ] No wiring issues or shorts

### Phase 3 Complete When:

- [ ] PAM8403 plays audio clearly
- [ ] No distortion or hardware issues
- [ ] ALSA device recognized by PulseAudio

### Phase 4 Complete When:

- [ ] Audio switch service runs without errors
- [ ] Switch UP → Audio plays through Bluetooth
- [ ] Switch DOWN → Audio plays through PAM8403
- [ ] Transitions happen within 0.5 seconds
- [ ] No audio glitches during switch

### Phase 5 Complete When:

- [ ] All R2D2 audio sources work with switch
- [ ] Gesture beeps route correctly
- [ ] Speech TTS routes correctly
- [ ] Recognition alerts route correctly

### Phase 6 Complete When:

- [ ] System stable for 1+ hour of switching
- [ ] Services survive restarts
- [ ] Configuration persists after reboot

---

## Reference Scripts

### GPIO Switch Test Script

**Location:** `~/dev/r2d2/scripts/test_gpio_switch.py`

**Purpose:** Isolated testing of switch hardware before integration

**Usage:**
```bash
python3 ~/dev/r2d2/scripts/test_gpio_switch.py
```

### Audio Switch Service

**Location:** `~/dev/r2d2/scripts/audio_switch_service.py`

**Purpose:** Production service that monitors GPIO and switches audio

**Control:**
```bash
sudo systemctl start r2d2-audio-switch.service
sudo systemctl stop r2d2-audio-switch.service
sudo systemctl status r2d2-audio-switch.service
```

### Manual Audio Switch

**Location:** `~/dev/r2d2/scripts/audio_switch.sh`

**Purpose:** Manual software-based audio switching

**Usage:**
```bash
# Switch to Bluetooth
~/dev/r2d2/scripts/audio_switch.sh bluetooth

# Switch to PAM8403
~/dev/r2d2/scripts/audio_switch.sh pam8403

# Check current mode
~/dev/r2d2/scripts/audio_switch.sh status
```

---

## Current Status (January 2, 2026)

| Component | Status | Notes |
|-----------|--------|-------|
| **FreeBuds 4i Bluetooth** | ✅ Working | A2DP profile active, paired and connected |
| **Software Switcher** | ✅ Ready | Manual script created and tested |
| **GPIO Test Script** | ✅ Created | Ready for hardware testing |
| **Audio Switch Service** | ✅ Created | Ready for deployment |
| **PAM8403 Speaker** | ❌ Disconnected | Hardware repair needed before testing |
| **Physical Switch** | ⏳ Pending | Install after PAM8403 repair |

---

## Next Steps

1. **Repair PAM8403 Hardware**
   - Inspect solder joints
   - Test with multimeter
   - Verify ALSA device appears
   - Test audio playback

2. **Install Physical Switch**
   - Wire according to diagram
   - Run GPIO test script
   - Verify readings

3. **Deploy Audio Switch Service**
   - Enable systemd service
   - Test both positions
   - Verify automatic switching

4. **Integration Testing**
   - Test with all R2D2 audio sources
   - Verify gesture/speech systems
   - Long-term stability testing

---

## Related Documentation

- [`261_BLUETOOTH_AUDIO_REFERENCE.md`](261_BLUETOOTH_AUDIO_REFERENCE.md) - Bluetooth audio setup
- [`262_BLUETOOTH_AUDIO_OPEN_POINTS.md`](262_BLUETOOTH_AUDIO_OPEN_POINTS.md) - PAM8403 hardware issues
- [`200_SPEECH_SYSTEM_REFERENCE.md`](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture
- [`001_ARCHITECTURE_OVERVIEW.md`](001_ARCHITECTURE_OVERVIEW.md) - System overview

---

**Last Updated:** January 2, 2026  
**Next Review:** After PAM8403 hardware repair

