# Audio Notification System Debug Findings

**Date:** December 15, 2025  
**Issue:** No audio beeps playing when person recognition state changes  
**Status:** ✅ Root Cause Identified and Fixed

## Root Cause

The audio notification system was failing silently because:

1. **ffplay ALSA Device Syntax Error**: The `audio_player.py` script was using `-ao alsa:device=hw:1,0` syntax, which ffplay does not support. This caused ffplay to fail with "Option not found" error.

2. **Silent Failure**: The subprocess errors were being suppressed by `stderr=subprocess.DEVNULL`, so failures were not visible in logs.

3. **Low Volume**: The default volume was set to 0.05 (5%), which is very quiet and may not be audible.

## Fixes Applied

### 1. Fixed ffplay ALSA Device Syntax (`audio_player.py`)

**Problem:** ffplay doesn't support `-ao alsa:device=hw:1,0` syntax.

**Solution:** Removed the device specification from ffplay command. The system's `~/.asoundrc` file already configures the default ALSA device to use `hw:1,0`, so ffplay will use the default device automatically.

**Changes:**
- Removed `-ao alsa:device=DEVICE` from ffplay command
- ffplay now uses the default ALSA device configured in `~/.asoundrc`

### 2. Enhanced Error Logging (`audio_notification_node.py`)

**Problem:** Subprocess errors were hidden, making debugging difficult.

**Solution:** Added better error logging and process status checking.

**Changes:**
- Changed "🔊 Playing" log message from DEBUG to INFO level
- Added process PID logging
- Added error capture for immediate subprocess failures
- Added exception logging with stack traces

### 3. Volume Adjustment

**Recommendation:** Increase volume from default 0.05 (5%) to 0.3-0.5 (30-50%) for better audibility.

**Command:**
```bash
ros2 param set /audio_notification_node audio_volume 0.5
```

## Testing Results

### Verified Working Components

1. ✅ Service is running: `r2d2-audio-notification.service` is active
2. ✅ Node is running: `/audio_notification_node` is active
3. ✅ Audio files exist: Both MP3 files found in install directory
4. ✅ Audio player script exists: `audio_player.py` is present
5. ✅ ffplay is installed: `/usr/bin/ffplay` available
6. ✅ ALSA hardware works: Test WAV file plays successfully
7. ✅ State machine triggers: Recognition events are logged
8. ✅ Volume parameter: Can be set via ROS 2 parameters

### Issues Found

1. ❌ ffplay command fails silently with ALSA device syntax error
2. ❌ Subprocess errors hidden by `stderr=subprocess.DEVNULL`
3. ⚠️ Volume too low (5% default)

## Next Steps

1. **Restart the service** to apply fixes:
   ```bash
   sudo systemctl restart r2d2-audio-notification.service
   ```

2. **Increase volume** (if not already done):
   ```bash
   ros2 param set /audio_notification_node audio_volume 0.5
   ```

3. **Test audio playback**:
   ```bash
   # Trigger recognition manually
   ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: no_person}"
   sleep 2
   ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: severin}"
   ```

4. **Monitor logs** for audio playback confirmation:
   ```bash
   journalctl -u r2d2-audio-notification.service -f | grep -E "Playing|audio|Error"
   ```

5. **Verify ffplay process** is spawned:
   ```bash
   ps aux | grep ffplay | grep -v grep
   ```

## Expected Behavior After Fix

- When person is recognized: "🔊 Playing RECOGNITION audio" log message appears
- ffplay process spawns and plays audio file
- Audio is audible at 30-50% volume
- No errors in service logs

## Files Modified

1. `dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_player.py`
   - Removed unsupported `-ao alsa:device=` syntax for ffplay
   - Simplified to use default ALSA device

2. `dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/audio_notification_node.py`
   - Enhanced error logging (DEBUG → INFO for playback messages)
   - Added process status checking
   - Added error capture for immediate failures

## Verification Commands

```bash
# Check service status
sudo systemctl status r2d2-audio-notification.service

# Check node is running
ros2 node list | grep audio_notification_node

# Check volume setting
ros2 param get /audio_notification_node audio_volume

# Monitor logs in real-time
journalctl -u r2d2-audio-notification.service -f

# Test audio playback directly
python3 ~/dev/r2d2/ros2_ws/install/r2d2_audio/lib/python3.10/site-packages/r2d2_audio/audio_player.py \
  /home/severin/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3 \
  0.5 hw:1,0
```

## Summary

The root cause was the unsupported ffplay ALSA device syntax. The fix removes the device specification and relies on the system's default ALSA device configuration. Enhanced logging will help diagnose any future issues. After restarting the service and increasing the volume, audio should play correctly.

