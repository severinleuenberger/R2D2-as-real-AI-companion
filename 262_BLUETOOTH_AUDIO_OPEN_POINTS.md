# Bluetooth Audio - Open Points & Next Steps

**Date:** January 8, 2026  
**Status:** Both Bluetooth and PAM8403 speaker working, physical switch pending

---

## Current Status

✅ **Working:**
- Bluetooth audio output via PulseAudio (A2DP mode, 44.1kHz stereo)
- PAM8403 speaker hardware operational (new board installed Jan 8, 2026)
- Speech TTS plays through both Bluetooth and PAM8403
- Audio notification beeps work on both outputs
- Gesture feedback sounds work on both outputs
- Services configured with PulseAudio environment variables
- Software audio switcher functional (`audio_switch.sh`)
- ALSA mixer routing auto-configured on boot

⏳ **In Progress:**
- Physical GPIO switch for hardware-based output selection (pending installation)

---

## Resolved Issues

### ✅ PAM8403 Speaker Hardware (RESOLVED - January 8, 2026)

**Previous Issue:** PAM8403 speaker module non-functional (old board damaged)

**Resolution:**
- New PAM8403 board installed and connected to J511 audio header
- Connected J511 Pin 9 (HPO_L) → PAM8403 RIN input
- Connected J511 Pin 2 (AGND) → PAM8403 GND
- ALSA mixer routing configured: ADMAIF1 → I2S6 → J511 output
- Auto-configuration service deployed: `r2d2-audio-routing.service`
- PulseAudio sink: `alsa_output.platform-sound.analog-stereo`

**Testing Results:**
- ✅ Clear audio playback through 8Ω speaker
- ✅ R2D2 beeps and TTS working perfectly
- ✅ Configuration persists after reboot
- ✅ No distortion or hardware issues

**Key Learning:**
- Jetson AGX Orin uses complex ALSA mixer routing (different from Nano)
- J511 header provides analog headphone output (not direct I2S)
- Mixer controls (numid=1218, 310, 1331) must be configured on boot

---

### ✅ Bluetooth A2DP Profile (WORKING)

**Previous Issue:** Only HFP profile available, not A2DP

**Current State:**
- FreeBuds 4i (MAC: `28:54:71:BB:C6:53`) connects in A2DP mode
- PulseAudio sink: `bluez_sink.28_54_71_BB_C6_53.a2dp_sink`
- Audio quality: 44.1kHz stereo (high quality)
- Works reliably for speech and audio notifications

**Status:** No longer an open issue

---

## Remaining Open Points

### 1. Physical GPIO Audio Switch (In Progress)

**Current State:** Software switching works, but manual command required

**Goal:** Hardware toggle switch for instant audio output selection

**Implementation Plan:**
- Install SPST toggle switch with 10kΩ pull-up on GPIO 17
- Deploy `r2d2-audio-switch.service` for automatic monitoring
- Switch UP → Bluetooth, Switch DOWN → PAM8403
- No software intervention needed during use

**Status:** Hardware components available, ready for installation

**See:** [`280_PAM8403_AUDIO_SWITCH_TEST_PLAN.md`](280_PAM8403_AUDIO_SWITCH_TEST_PLAN.md) for detailed installation procedure

---

## Next Steps (Priority Order)

### High Priority

1. **Install Physical GPIO Switch**
   - Wire toggle switch to GPIO 17 with pull-up resistor
   - Test switch detection with `test_gpio_switch.py`
   - Deploy `r2d2-audio-switch.service`
   - Verify automatic switching works reliably

### Medium Priority

2. **Audio Quality Testing**
   - Compare Bluetooth vs PAM8403 audio quality
   - Measure latency differences
   - Document trade-offs for each output

3. **Long-term Stability Testing**
   - Monitor both audio outputs for 24+ hours
   - Test rapid switching scenarios
   - Verify no audio glitches or dropouts

### Low Priority

4. **Bluetooth Device Management**
   - Auto-reconnect on disconnect
   - Status monitoring script
   - Handle multiple paired devices

---

## Related Documentation

- [`280_PAM8403_AUDIO_SWITCH_TEST_PLAN.md`](280_PAM8403_AUDIO_SWITCH_TEST_PLAN.md) - Complete test plan and installation guide
- [`261_BLUETOOTH_AUDIO_REFERENCE.md`](261_BLUETOOTH_AUDIO_REFERENCE.md) - Bluetooth setup and configuration
- [`200_SPEECH_SYSTEM_REFERENCE.md`](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture
- [`002_HARDWARE_REFERENCE.md`](002_HARDWARE_REFERENCE.md) - Hardware connections

---

**Last Updated:** January 8, 2026

**Desired:** Automatic detection and selection of available audio output.

**Possible Implementation:**
- Detect if Bluetooth device is connected
- Check if PAM8403 produces audio (test tone)
- Auto-select best available output
- Fallback chain: Bluetooth → PAM8403 → System default

**Configuration Option:**
```yaml
# Auto-detect best available output
sink_device: 'auto'

# Or explicit selection
sink_device: 'pulse'  # Bluetooth
sink_device: 'default'  # PAM8403
```

---

### 4. Bluetooth Device MAC Address Stability

**Issue:** Bluetooth device MAC address changed from `54:D5:64:7C:21:E8` to `CC:FF:90:45:1A:3D`.

**Possible Causes:**
- Device firmware update
- Bluetooth pairing reset
- Multiple devices with similar name

**Impact:**
- Documentation references old MAC address
- Scripts may need updating
- Default sink configuration may break

**Solution:**
- Use device name/alias instead of MAC address where possible
- Document current MAC address
- Create helper script to find and connect Bluetooth device by name

---

## Next Steps (Priority Order)

### High Priority

1. **Fix A2DP Profile Connection**
   - Research PulseAudio A2DP activation
   - Test with different Bluetooth device
   - Improve audio quality from 16kHz mono to 44.1kHz stereo

2. **Document Current Bluetooth Setup**
   - Update MAC address references
   - Document HFP vs A2DP limitations
   - Add troubleshooting for profile issues

### Medium Priority

3. **PAM8403 Hardware Diagnosis**
   - Visual inspection of solder joints
   - Multimeter testing
   - Decide: repair or replace

4. **Audio Output Auto-Selection**
   - Implement detection logic
   - Add configuration option
   - Test fallback chain

### Low Priority

5. **Bluetooth Device Management Script**
   - Helper script to find/connect by name
   - Auto-reconnect on disconnect
   - Status monitoring

6. **Audio Quality Testing**
   - Compare HFP vs A2DP quality
   - Measure latency differences
   - Document trade-offs

---

## Testing Recommendations

### A2DP Profile Test

```bash
# 1. Disconnect and reconnect
bluetoothctl disconnect CC:FF:90:45:1A:3D
bluetoothctl connect CC:FF:90:45:1A:3D

# 2. Check profiles after reconnect
pactl list cards | grep -A 30 "bluez_card"

# 3. Try different connection methods
# (via bluetoothctl vs PulseAudio auto-connect)
```

### PAM8403 Hardware Test

```bash
# 1. Visual inspection
# - Check solder joints on audio input
# - Check PAM8403 chip for heat damage
# - Check speaker wire connections

# 2. Continuity test
# - Test audio input path with multimeter
# - Verify power connections
# - Check for shorts

# 3. Test with known-good module (if available)
```

### Audio Quality Comparison

```bash
# Test HFP quality
pactl set-default-sink bluez_sink.CC_FF_90_45_1A_3D.handsfree_head_unit
paplay test_audio.wav

# Test A2DP quality (when available)
pactl set-default-sink bluez_sink.CC_FF_90_45_1A_3D.a2dp_sink
paplay test_audio.wav

# Compare: frequency response, clarity, latency
```

---

## Related Documentation

- [261_BLUETOOTH_AUDIO_REFERENCE.md](261_BLUETOOTH_AUDIO_REFERENCE.md) - Bluetooth setup and configuration
- [200_SPEECH_SYSTEM_REFERENCE.md](200_SPEECH_SYSTEM_REFERENCE.md) - Speech system architecture
- [203_SPEECH_SYSTEM_TROUBLESHOOTING.md](203_SPEECH_SYSTEM_TROUBLESHOOTING.md) - General troubleshooting

---

**Last Updated:** December 29, 2025

