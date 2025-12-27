# R2D2 Audio Baseline Test Results

> **Completed**: 2025-12-27

## Test Information

**Test Date**: 2025-12-27  
**Hardware**: PAM8403 + 8Ω Speaker (J511 audio via `hw:1,0`)  
**Tester**: Severin  
**Test Method**: Interactive testing using `ffplay` with `-af volume=X`

## Summary

| Audio Source | Min Audible | Default | Max Safe | Optimal Range |
|--------------|-------------|---------|----------|---------------|
| Recognition beep | 0.01 | 0.1 | 1.0 | 0.05 - 0.3 |
| Loss beep | 0.01 | 0.1 | 0.7 | 0.2 - 0.3 |
| Gesture ack | 0.05 | 0.1 | 0.7 | 0.1 - 0.3 |
| Session start | 0.01 | 0.1 | 0.5 | 0.1 - 0.3 |
| Session stop | 0.05 | 0.1 | 0.5 | 0.2 - 0.3 |

## Key Findings

### Distortion Threshold
- **Distortion starts at volume 0.7-1.0** for most audio sources
- Session start/stop beeps distort earlier (at 0.5-0.7)
- Recognition beep handles higher volumes better (up to 1.0)

### Volume Ranges
- **Minimum audible**: 0.01-0.05 depending on audio source
- **Comfortable range**: 0.05 - 0.3
- **Maximum safe (no distortion)**: 0.7

### Recommendations Applied

1. **Maximum volume cap**: `0.7` (prevents distortion at all knob positions)
2. **Default volume**: `0.1` (was 0.02, now clearly audible)
3. **Default master volume**: `0.35` (middle of 0.0-0.7 range)

## Volume Curve Configuration

Based on testing:

- [x] Linear mapping selected (provides intuitive control)
- [ ] Logarithmic mapping (not needed - linear works well)

### Dead Zone Settings

- **Low dead zone**: 5% (map 0-5% knob position to mute)
- **High dead zone**: 5% (map 95-100% knob position to max safe)

## Hardware Notes

### PulseAudio vs ALSA
- Direct ALSA (`hw:1,0`) does not support mixer controls
- PulseAudio pass-through works reliably
- `ffplay` uses PulseAudio by default, which works correctly

### Audio Device
- Device: `hw:1,0` (PAM8403 via J511)
- No ALSA mixer controls available on this device
- Volume must be controlled in software (ffplay -af volume=X)

## Test Commands Used

```bash
# Stop conflicting services first
sudo systemctl stop r2d2-speech-node.service
sudo systemctl stop r2d2-audio-notification.service

# Test audio at different volumes
ffplay -nodisp -autoexit -af volume=0.1 'Voicy_R2-D2 - 2.mp3'
ffplay -nodisp -autoexit -af volume=0.3 'Voicy_R2-D2 - 2.mp3'
ffplay -nodisp -autoexit -af volume=0.7 'Voicy_R2-D2 - 2.mp3'

# Restart services after testing
sudo systemctl start r2d2-speech-node.service
sudo systemctl start r2d2-audio-notification.service
```

## Configuration Applied

These results have been applied to:

1. **`audio_params.yaml`**:
   - `audio_volume: 0.1` (was 0.02)
   - `master_volume_default: 0.35` (middle of safe range)
   - `max_volume_cap: 0.7` (distortion threshold)

2. **`volume_control_node.py`**:
   - Loads calibration from `~/.r2d2/adc_calibration.json`
   - Maps knob position to 0.0 - 0.7 volume range
   - 5% dead zones at min/max positions

## Change History

| Date | Tester | Changes |
|------|--------|---------|
| 2025-12-27 | Severin | Initial baseline test completed |
| 2025-12-27 | System | Applied results to audio_params.yaml and volume_control_node.py |
