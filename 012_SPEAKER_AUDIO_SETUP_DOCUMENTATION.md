# Speaker & Audio Hardware Setup Documentation

**Date:** December 8, 2025  
**Hardware:** NVIDIA Jetson AGX Orin 64GB with PAM8403 Class-D amplifier + 8Ω / 3W speaker  
**Scope:** Physical hardware wiring, ALSA configuration, and audio output verification

---

## Executive Summary

This document covers the **hardware setup** for R2D2's audio output system, including physical wiring, ALSA configuration, and basic audio testing. This is the foundation that enables all audio notifications and alerts.

**For ROS 2 audio integration and notifications**, see: [`010_PERSON_RECOGNITION_STATUS.md`](010_PERSON_RECOGNITION_STATUS.md)

---

## Hardware Architecture

The Jetson AGX Orin has two audio subsystems:
1. **HDA (High Definition Audio)** - Card 0 - HDMI outputs only (no analog)
2. **APE (Audio Processing Engine)** - Card 1 - I2S interface (direct analog output)

**Current Configuration:** Using APE Card 1 with I2S interface to drive the HPO_L (left channel) pin on J511 header via PAM8403 amplifier.

**Challenge:** NVIDIA's Jetson doesn't ship with pre-configured analog output codec, but the hardware exists. We route I2S audio directly to the physical HPO pin using ALSA.

**Solution:** Simple **ALSA configuration** with APE I2S device for direct HPO output control.

---

## Hardware Wiring

**Jetson AGX Orin → PAM8403 → Speaker**

```
Jetson 40-pin header (J30):
  Pin 2  (5V)   → PAM8403 +5V (power supply)
  Pin 6  (GND)  → PAM8403 GND (power ground)

Jetson audio panel header (J511):
  Pin 9  (HPO_L, left analog audio out) → PAM8403 LIN (left audio input)
  Pin 2  (AGND, audio ground)          → PAM8403 GND (audio signal ground)

PAM8403 speaker output (class-D amplifier):
  L+ → speaker + (wired correctly)
  L− → speaker − (wired correctly)

Amplifier Specs:
  - Input: 3.3V max (Jetson HPO_L output)
  - Gain: 23dB fixed (configurable via PAM8403 GAIN pin if needed)
  - Output: 3W @ 8Ω (adequate for test speaker)
  - Supply voltage: 5V ✓ (from Jetson Pin 2)
```

---

## ALSA Configuration

### Step 1: Install ALSA Utilities

```bash
sudo apt update
sudo apt install -y alsa-utils pulseaudio-utils
```

### Step 2: Create ALSA Configuration File

Create `/etc/asound.conf`:

```bash
sudo tee /etc/asound.conf > /dev/null << 'EOF'
# R2D2 Audio Configuration: PAM8403 Speaker via I2S (Card 1)

pcm.!default {
    type asym
    playback.pcm "speaker_out"
    capture.pcm "speaker_in"
}

pcm.speaker_out {
    type dmix
    ipc_key 1234
    slave {
        pcm "hw:1,0"           # Card 1 (APE), Device 0 (I2S)
        rate 44100
        channels 2
        period_size 4096
        buffer_size 65536
    }
}

pcm.speaker_in {
    type hw
    card 1
    device 0
}

ctl.!default {
    type hw
    card 1
}
EOF
```

### Step 3: Verify Configuration

```bash
aplay -l  # Should show Card 1 (APE)
```

---

## Testing Audio Output

### Simple Test

```bash
# Generate test tone
ffmpeg -f lavfi -i sine=f=1000:d=3 -ar 44100 -ac 2 /tmp/test.wav -y

# Play test tone
aplay -D hw:1,0 /tmp/test.wav
```

**Expected:** You should hear a 1kHz beep for 3 seconds.

---

## Troubleshooting

### No Sound from Speaker

1. Check PAM8403 power supply (Jetson Pin 2 → +5V)
2. Check speaker connection to PAM8403 outputs
3. Check J511 header wiring (Pin 9: HPO_L, Pin 2: AGND)
4. Verify ALSA device: `aplay -l` should show Card 1

### ALSA Errors

```bash
# Check kernel logs
dmesg | grep -i "i2s\|asoc\|audio" | tail -20
```

---

## Verification Checklist

- [ ] ALSA tools installed
- [ ] `/etc/asound.conf` created
- [ ] `aplay -l` shows Card 1 (APE)
- [ ] Test tone plays successfully
- [ ] No errors in `dmesg`

---

**Document Version:** 1.0  
**Last Updated:** December 8, 2025  
**Related:** [`010_PERSON_RECOGNITION_STATUS.md`](010_PERSON_RECOGNITION_STATUS.md) for ROS 2 integration
