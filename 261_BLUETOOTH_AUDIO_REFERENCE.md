# R2D2 Bluetooth Audio Reference

> **Last Updated**: 2026-01-02  
> **Status**: Production (alternative to PAM8403 speaker)  
> **Mono Support**: ✅ Single earbud use supported (L+R channels duplicated)

## Overview

R2D2 supports Bluetooth audio output as an alternative to the built-in PAM8403 speaker. This enables wireless audio playback through Bluetooth headphones, earbuds, or speakers.

**Supported Audio Sources:**
- Speech TTS (OpenAI Realtime API responses)
- MP3 notification beeps (recognition, loss alerts)
- Gesture feedback sounds

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Audio Output Options                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐         ┌──────────────┐         ┌──────────────┐ │
│  │ speech_node  │         │audio_notif...│         │gesture_intent│ │
│  │  (PyAudio)   │         │  (ffplay)    │         │  (ffplay)    │ │
│  └──────┬───────┘         └──────┬───────┘         └──────┬───────┘ │
│         │                        │                        │         │
│         └────────────────────────┼────────────────────────┘         │
│                                  │                                   │
│                                  ▼                                   │
│                    ┌─────────────────────────┐                       │
│                    │      PulseAudio         │                       │
│                    │  (sink_device: pulse)   │                       │
│                    └────────────┬────────────┘                       │
│                                 │                                    │
│              ┌──────────────────┼──────────────────┐                 │
│              ▼                                     ▼                 │
│   ┌─────────────────────┐             ┌─────────────────────┐       │
│   │  PAM8403 Speaker    │             │  Bluetooth Device   │       │
│   │  (hw:APE,0)         │             │  (bluez_sink.XX)    │       │
│   │  [ALSA Direct]      │             │  [PulseAudio]       │       │
│   └─────────────────────┘             └─────────────────────┘       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Mono Audio Support (Single Earbud Use)

**Status:** ✅ Fully Supported (January 2, 2026)

All audio sources output **dual-mono stereo** - identical content on both L+R channels. This ensures users with only one earbud (left OR right) hear all audio without loss.

**Implementation:**
- **Speech TTS:** `AudioPlayback` class duplicates mono to stereo (`audio_stream.py` line 527-529)
- **Gesture Beeps:** `ffplay` uses pan filter `pan=stereo|c0=c0|c1=c0` (`gesture_intent_node.py` line 611)
- **Audio Files:** R2D2 beeps are mono (1 channel), automatically duplicated to both ears

**Testing:**
```bash
# Test with single earbud (left OR right) - should hear full audio
ffplay -autoexit -nodisp -af 'pan=stereo|c0=c0|c1=c0' \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 12.mp3
```

**Known Issue:** Stop beep (fist gesture) may not be audible due to timing - see [`999_FIX_STOP_BEEP_TIMING.md`](999_FIX_STOP_BEEP_TIMING.md)

---

### Audio Routing Modes

| Mode | Config Value | Route | Use Case |
|------|--------------|-------|----------|
| **ALSA Direct** | `sink_device: 'default'` | PyAudio → ALSA → PAM8403 | Built-in speaker |
| **PulseAudio** | `sink_device: 'pulse'` | PyAudio → PulseAudio → Default Sink | Bluetooth/flexible |

---

## Quick Start

### 1. Pair Bluetooth Device (One-time)

```bash
# Enter Bluetooth control
bluetoothctl

# Enable scanning
scan on

# Wait for device to appear, then pair (replace XX:XX... with MAC)
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX

# Exit
exit
```

### 2. Verify Bluetooth as Default Sink

```bash
# Check default sink
pactl get-default-sink

# List all sinks
pactl list sinks short

# Set Bluetooth as default (if not automatic)
pactl set-default-sink bluez_sink.XX_XX_XX_XX_XX_XX.a2dp_sink
```

### 3. Test Audio

```bash
# Test with system sound
paplay /usr/share/sounds/alsa/Front_Center.wav

# Test with beep
ffplay -autoexit -nodisp -f lavfi "sine=frequency=800:duration=0.5"
```

---

## Configuration

### Speech System (PyAudio)

**File:** `ros2_ws/src/r2d2_speech/config/speech_params.yaml`

```yaml
# For Bluetooth (via PulseAudio)
sink_device: 'pulse'

# For PAM8403 speaker (ALSA direct)
# sink_device: 'default'
```

After changing, rebuild and restart:

```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_speech
sudo systemctl restart r2d2-speech-node
```

### SystemD Services Environment

All audio services require PulseAudio environment variables to access the user's audio session:

```bash
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

**Services requiring these variables:**
- `r2d2-speech-node.service`
- `r2d2-gesture-intent.service`
- `r2d2-audio-notification.service` (via start script)

---

## Services Configuration

### Speech Node Service

**File:** `/etc/systemd/system/r2d2-speech-node.service`

```ini
[Service]
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

### Speech Start Script

**File:** `scripts/start/start_speech_node.sh`

```bash
# PulseAudio environment for user session audio
export XDG_RUNTIME_DIR=/run/user/1000
export PULSE_SERVER=unix:/run/user/1000/pulse/native
```

### Gesture Intent Service

**File:** `/etc/systemd/system/r2d2-gesture-intent.service`

```ini
[Service]
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

### Audio Notification Start Script

**File:** `scripts/start/start_audio_notification.sh`

```bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native
```

---

## Switching Audio Output

### Switch to Bluetooth

1. Connect Bluetooth device:
   ```bash
   bluetoothctl connect XX:XX:XX:XX:XX:XX
   ```

2. Verify it's the default sink:
   ```bash
   pactl get-default-sink
   # Should show: bluez_sink.XX_XX_XX_XX_XX_XX.a2dp_sink
   ```

3. Ensure config uses PulseAudio:
   ```bash
   grep sink_device ~/dev/r2d2/ros2_ws/install/r2d2_speech/share/r2d2_speech/config/speech_params.yaml
   # Should show: sink_device: 'pulse'
   ```

4. Restart services:
   ```bash
   sudo systemctl restart r2d2-speech-node r2d2-gesture-intent r2d2-audio-notification
   ```

### Switch to PAM8403 Speaker

1. Change config to ALSA direct:
   ```bash
   # Edit source config
   nano ~/dev/r2d2/ros2_ws/src/r2d2_speech/config/speech_params.yaml
   # Change: sink_device: 'default'
   
   # Rebuild
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_speech
   ```

2. Restart services:
   ```bash
   sudo systemctl restart r2d2-speech-node r2d2-gesture-intent r2d2-audio-notification
   ```

---

## Troubleshooting

### Issue: No Audio on Bluetooth

**Symptom:** Beeps work but speech doesn't, or vice versa

**Check 1:** Verify PulseAudio environment in running process
```bash
cat /proc/$(pgrep -f "speech_node --ros-args" | head -1)/environ | tr '\0' '\n' | grep PULSE
# Should show: PULSE_SERVER=unix:/run/user/1000/pulse/native
```

**Check 2:** Verify sink_device configuration
```bash
grep sink_device ~/dev/r2d2/ros2_ws/install/r2d2_speech/share/r2d2_speech/config/speech_params.yaml
# Should show: sink_device: 'pulse'
```

**Check 3:** Check service logs for PulseAudio errors
```bash
journalctl -u r2d2-speech-node --since "5 minutes ago" | grep -i pulse
# Should NOT show: "Unable to connect: Connection refused"
```

**Fix:** Restart services after daemon-reload
```bash
sudo systemctl daemon-reload
sudo systemctl restart r2d2-speech-node r2d2-gesture-intent r2d2-audio-notification
```

### Issue: Bluetooth Device Not Connecting

**Symptom:** `bluetoothctl connect` fails

**Check:** Install Bluetooth PulseAudio module
```bash
sudo apt install -y pulseaudio-module-bluetooth
pulseaudio -k  # Restart PulseAudio
```

**Then reconnect:**
```bash
bluetoothctl
disconnect XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
```

### Issue: Audio Plays to Wrong Device

**Symptom:** Audio plays to PAM8403 instead of Bluetooth

**Check:** Default sink
```bash
pactl get-default-sink
```

**Fix:** Set Bluetooth as default
```bash
pactl set-default-sink bluez_sink.XX_XX_XX_XX_XX_XX.a2dp_sink
```

### Issue: Services Can't Access PulseAudio

**Symptom:** Log shows "PulseAudio: Unable to connect: Connection refused"

**Root Cause:** Missing environment variables in systemd service

**Fix:** Add to service file:
```ini
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl restart <service-name>
```

---

## Tested Devices

| Device | Status | Notes |
|--------|--------|-------|
| HUAWEI FreeBuds 5 | ✅ Working | Used for initial testing |

---

## Related Documentation

- [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture
- [260_VOLUME_CONTROL_REFERENCE.md](260_VOLUME_CONTROL_REFERENCE.md) - Volume control system
- [203_SPEECH_SYSTEM_TROUBLESHOOTING.md](203_SPEECH_SYSTEM_TROUBLESHOOTING.md) - General troubleshooting

---

## Technical Details

### Why PulseAudio Environment Variables?

SystemD services run in an isolated environment without access to the user's session. PulseAudio runs per-user, so services need explicit configuration to connect:

- `XDG_RUNTIME_DIR`: Points to user's runtime directory (`/run/user/1000`)
- `PULSE_SERVER`: Unix socket path for PulseAudio connection

### User ID Note

The configuration uses UID 1000 (user `severin`). If running as a different user, adjust accordingly:

```bash
# Find your UID
id -u

# Update service files with correct UID
```


