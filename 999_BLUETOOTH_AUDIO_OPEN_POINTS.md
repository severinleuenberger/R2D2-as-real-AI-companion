# Bluetooth Audio - Open Points & Next Steps

**Date:** December 29, 2025  
**Status:** Bluetooth audio working, PAM8403 speaker confirmed non-functional

---

## Current Status

✅ **Working:**
- Bluetooth audio output via PulseAudio
- Speech TTS plays through Bluetooth
- Audio notification beeps work on Bluetooth
- Gesture feedback sounds work on Bluetooth
- Services configured with PulseAudio environment variables
- Default sink set to Bluetooth device

❌ **Known Issues:**
- PAM8403 speaker hardware is non-functional (likely damaged from soldering)
- Bluetooth device only connects in HFP (handsfree) mode, not A2DP (high-quality stereo)
  - Current: `bluez_sink.CC_FF_90_45_1A_3D.handsfree_head_unit` (16kHz mono)
  - Desired: `bluez_sink.CC_FF_90_45_1A_3D.a2dp_sink` (44.1kHz/48kHz stereo)

---

## Open Points

### 1. A2DP Profile Not Available

**Issue:** Huawei FreeBuds 5i only connects in HFP (handsfree) mode, not A2DP (high-quality stereo).

**Current State:**
- Device connects successfully
- Only `handsfree_head_unit` profile available
- Audio quality: 16kHz mono (telephone quality)
- Works but not optimal for speech playback

**Possible Causes:**
- Bluetooth codec negotiation issue
- PulseAudio module configuration
- Device firmware limitation
- Missing A2DP profile support in PulseAudio

**Investigation Steps:**
```bash
# Check available profiles
pactl list cards | grep -A 30 "bluez_card"

# Check PulseAudio Bluetooth module
pactl list modules short | grep bluez

# Check device capabilities
bluetoothctl info CC:FF:90:45:1A:3D

# Try forcing A2DP
pactl set-card-profile bluez_card.CC_FF_90_45_1A_3D a2dp_sink
```

**Next Steps:**
- Research PulseAudio A2DP profile activation
- Test with different Bluetooth device to isolate issue
- Check if `pulseaudio-module-bluetooth` needs reconfiguration
- Consider alternative Bluetooth stack (PipeWire?)

---

### 2. PAM8403 Speaker Hardware Repair

**Issue:** PAM8403 speaker module produces no audio output despite:
- ALSA device opens successfully (`hw:APE,0`)
- Audio data sent without errors
- Mixer controls show unmuted, reasonable volume levels
- Only right channel wired (left channel not connected)

**Diagnosis:**
- Software layer: ✅ Working (ALSA, mixer, audio stream)
- Hardware layer: ❌ Likely damaged (soldering work mentioned)

**Possible Hardware Issues:**
- PAM8403 chip damaged from heat during soldering
- Cold solder joint on audio input
- Broken trace on PCB
- Short circuit
- Speaker connection issue

**Next Steps:**
- Visual inspection of solder joints with magnifier
- Multimeter continuity test on audio path
- Test with known-good PAM8403 module (if available)
- Consider replacement module if repair not feasible

**Workaround:**
- Bluetooth audio works as alternative
- No immediate need for PAM8403 if Bluetooth is acceptable

---

### 3. Audio Output Selection Automation

**Current State:** Manual configuration required to switch between Bluetooth and PAM8403.

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

