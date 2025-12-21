# R2D2 Perception and Status System - Troubleshooting Guide
## Debug Procedures for Face Recognition, Gestures, and Status Management

**Date:** December 21, 2025  
**Purpose:** Comprehensive troubleshooting for perception and status system  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Systematic Debug Procedure

Follow these steps in order when the system is not working:

### Step 1: Check Services

```bash
# Check all perception/status services
systemctl is-active r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent

# All should show "active"
# If any shows "inactive" or "failed":
sudo systemctl status r2d2-camera-perception --no-pager
sudo journalctl -u r2d2-camera-perception --since "5 minutes ago"
```

### Step 2: Check ROS2 Nodes

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list

# EXPECT to see:
# /camera_node or /oak_d_camera
# /image_listener
# /audio_notification_node
# /status_led_node
# /gesture_intent_node
```

### Step 3: Check Topics Publishing

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Person ID (face recognition)
timeout 3 ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: "target_person" when in front of camera

# Person status (RED/GREEN/BLUE)
timeout 3 ros2 topic echo /r2d2/audio/person_status --once
# EXPECT: {"status": "red",...} when recognized

# Gesture events
timeout 10 ros2 topic echo /r2d2/perception/gesture_event
# Make gesture, EXPECT: "index_finger_up" or "fist"

# Face count
timeout 3 ros2 topic echo /r2d2/perception/face_count --once
# EXPECT: 1 when face detected, 0 when no face
```

### Step 4: Test Hardware

**Camera:**
```bash
# Check USB connection
lsusb | grep Movidius
# EXPECT: 03e7:2485 Intel Movidius MyriadX

# Test camera in virtualenv
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"
# EXPECT: 1
```

**Audio:**
```bash
# Test manual beep
ffplay -nodisp -autoexit -af "volume=0.50" \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3
# Should hear R2D2 beep
```

**LED:**
```bash
# Check GPIO export
ls -la /sys/class/gpio/gpio17/
# Should exist and be accessible

# Manual test
echo 1 > /sys/class/gpio/gpio17/value  # ON
sleep 2
echo 0 > /sys/class/gpio/gpio17/value  # OFF
```

---

## Common Issues and Solutions

### Issue: Camera Not Working

**Symptoms:**
- No video topics published
- Node crashes on startup
- "No devices found" error

**Diagnosis:**
```bash
# 1. Check USB connection
lsusb | grep Movidius
# Should show: 03e7:2485

# 2. Check environment variable
echo $OPENBLAS_CORETYPE
# EXPECT: ARMV8

# 3. Check virtualenv
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print(f'DepthAI {dai.__version__}')"
# Should print version

# 4. Check service logs
sudo journalctl -u r2d2-camera-perception -n 50
```

**Solutions:**
1. **USB not detected:**
   - Try different USB port
   - Use direct port (not through hub)
   - Check cable quality
   
2. **"Illegal instruction" error:**
   ```bash
   echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Service fails to start:**
   ```bash
   # Check virtualenv in startup script
   cat ~/dev/r2d2/start_camera_perception.sh | grep depthai_env
   # Should source virtualenv
   ```

---

### Issue: Face Recognition Always Returns "unknown"

**Symptoms:**
- Person ID shows "unknown" when you're in front of camera
- Face count shows 1 but no recognition
- LED stays OFF (BLUE or GREEN)

**Diagnosis:**
```bash
# 1. Check recognition is enabled
ros2 param get /image_listener enable_face_recognition
# EXPECT: True

# 2. Check confidence scores
ros2 topic echo /r2d2/perception/face_confidence --once
# Lower is better, should be < 150 for recognized person
# If > 150, face not matching trained model

# 3. Check model exists
ls -la ~/dev/r2d2/data/face_recognition/models/*.xml
# Should show trained model files

# 4. Check PersonRegistry
cd ~/dev/r2d2/tests/face_recognition
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print([p['display_name'] for p in r.list_persons()])"
# Should list your person
```

**Solutions:**
1. **Threshold too strict:**
   ```bash
   # Lower threshold temporarily (accept weaker matches)
   ros2 param set /image_listener recognition_confidence_threshold 80.0
   
   # Test, if works, make permanent:
   nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml
   # Change: recognition_confidence_threshold: 80.0
   ```

2. **Model not trained well:**
   ```bash
   # Add more training images
   cd ~/dev/r2d2/tests/face_recognition
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   python3 train_manager.py
   # Select [2] Add additional pictures
   # Then [3] Retrain from existing
   ```

3. **Wrong lighting conditions:**
   - Train in similar lighting as production
   - Add images in various lighting conditions
   - Ensure face is well-lit during use

---

### Issue: No Audio Beeps Heard

**Symptoms:**
- Recognition/gestures work (topics show data)
- No audio feedback
- No "Hello!" or "Lost you!" beeps

**Diagnosis:**
```bash
# 1. Check audio files exist
ls -la ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/*.mp3
# Should show 4 files (2, 5, 16, 20)

# 2. Check volume
ros2 param get /audio_notification_node audio_volume
# Default: 0.02 (2%, very quiet)

# 3. Test manual playback
ffplay -nodisp -autoexit -af "volume=0.50" \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# 4. Check logs for "Playing audio"
sudo journalctl -u r2d2-audio-notification -f | grep -E "Playing|audio"

# 5. Check ALSA device
aplay -l | grep "card 1"
# Should show APE device
```

**Solutions:**
1. **Volume too quiet:**
   ```bash
   # Increase volume temporarily
   ros2 param set /audio_notification_node audio_volume 0.5
   
   # If works, make permanent:
   nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml
   # Change: audio_volume: 0.5
   
   cd ~/dev/r2d2/ros2_ws
   colcon build --packages-select r2d2_audio
   sudo systemctl restart r2d2-audio-notification
   ```

2. **Speaker not connected:**
   - Check PAM8403 wiring (see installation guide)
   - Verify 5V power to amplifier
   - Check audio signal from J511 header

3. **ALSA misconfigured:**
   ```bash
   # Check/recreate /etc/asound.conf
   sudo nano /etc/asound.conf
   # Should contain pcm.!default pointing to hw:1,0
   ```

4. **Beeps only on transitions:**
   - "Hello!" beep: BLUE → RED (must walk away first)
   - "Lost you!" beep: RED → BLUE (after 15s timer)
   - No beeps on GREEN state
   - Walk away, wait 20s, return to trigger beeps

---

### Issue: Gestures Not Working

**Symptoms:**
- Gesture events not published
- Hand detected but no gesture recognition
- Model accuracy too low

**Diagnosis:**
```bash
# 1. Check gesture recognition enabled
ros2 param get /image_listener enable_gesture_recognition
# EXPECT: True

# 2. Check person status
ros2 topic echo /r2d2/audio/person_status --once | grep status
# EXPECT: "red" (gestures only work when RED)

# 3. Monitor gesture events
timeout 10 ros2 topic echo /r2d2/perception/gesture_event
# Make gestures, should see events

# 4. Check model exists
ls -la ~/dev/r2d2/data/gesture_recognition/models/*.pkl
# Should show trained model files

# 5. Check logs
sudo journalctl -u r2d2-camera-perception -f | grep -i gesture
```

**Solutions:**
1. **Not in RED status:**
   - Stand in front of camera
   - Wait for LED to turn ON
   - Verify person recognized
   - Then try gestures

2. **Model not trained:**
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   source ~/depthai_env/bin/activate
   export OPENBLAS_CORETYPE=ARMV8
   python3 train_manager.py
   # Select [8] Train gestures for person
   # Follow capture instructions
   ```

3. **Model accuracy low:**
   ```bash
   # Add more gesture training images
   python3 train_manager.py
   # Select [9] Add additional gesture pictures
   # Then [10] Train gesture model from existing
   # Then [11] Test gesture classifier
   ```

4. **Hand not detected:**
   - Move hand closer to camera (arm's length)
   - Ensure good lighting
   - Keep hand in camera frame
   - Make gesture more pronounced

---

### Issue: Gestures Detected But Ignored

**Symptoms:**
- Gesture events published
- Gesture intent node ignores them
- No service calls made

**Diagnosis:**
```bash
# Monitor gesture intent logs
sudo journalctl -u r2d2-gesture-intent -f

# Make gesture, check for:
# "Gesture ignored: person_status=..." → Not RED
# "Start gesture ignored: cooldown" → Too soon
# "Service not available" → Speech node not ready
```

**Solutions:**
1. **Person not RED:**
   ```bash
   # Verify status
   ros2 topic echo /r2d2/audio/person_status --once | grep status
   # Must be "red", stand in camera
   ```

2. **Cooldown active:**
   - Wait 5 seconds between start gestures
   - Wait 3 seconds between stop gestures

3. **Speech service not available:**
   ```bash
   # Check speech node running
   ros2 node list | grep speech_node
   
   # Check services
   ros2 service list | grep speech
   ```

---

### Issue: RED Status Unstable/Flickering

**Symptoms:**
- LED flickers ON/OFF rapidly
- Status changes between RED and BLUE/GREEN quickly
- Timer doesn't stay in RED

**Diagnosis:**
```bash
# 1. Monitor RED timer resets
sudo journalctl -u r2d2-audio-notification -f | grep "Timer reset"
# Should see frequent resets while in camera

# 2. Check face recognition rate
ros2 topic hz /r2d2/perception/person_id
# EXPECT: ~6.5 Hz

# 3. Check confidence scores
ros2 topic echo /r2d2/perception/face_confidence
# Should be < 150 consistently

# 4. Check rolling window parameters
ros2 param get /audio_notification_node red_entry_match_threshold
ros2 param get /audio_notification_node red_entry_window_seconds
# EXPECT: 3 and 1.0
```

**Solutions:**
1. **Face recognition unreliable:**
   - Retrain with more images
   - Lower confidence threshold
   - Improve lighting conditions

2. **Rolling window too strict:**
   ```bash
   # Lower match threshold (2 matches instead of 3)
   ros2 param set /audio_notification_node red_entry_match_threshold 2
   
   # Or increase window size
   ros2 param set /audio_notification_node red_entry_window_seconds 1.5
   ```

3. **RED timer too short:**
   ```bash
   # Increase timeout
   ros2 param set /audio_notification_node red_status_timeout_seconds 20.0
   ```

---

### Issue: LED Not Working

**Symptoms:**
- LED never turns ON
- GPIO errors in logs
- Permissions denied

**Diagnosis:**
```bash
# 1. Check GPIO exported
ls -la /sys/class/gpio/gpio17/
# Should exist

# 2. Check permissions
cat /sys/class/gpio/gpio17/value
# Should read 0 or 1

# 3. Manual test
echo 1 | sudo tee /sys/class/gpio/gpio17/value
# LED should turn ON

# 4. Check LED node running
ros2 node list | grep status_led
# EXPECT: /status_led_node

# 5. Check logs
sudo journalctl -u r2d2-audio-notification -f | grep LED
```

**Solutions:**
1. **GPIO not exported:**
   ```bash
   # Export GPIO
   echo 17 | sudo tee /sys/class/gpio/export
   echo out | sudo tee /sys/class/gpio/gpio17/direction
   
   # Set permissions
   sudo chown severin:severin /sys/class/gpio/gpio17/value
   ```

2. **Wiring issue:**
   - Check connections (see installation guide)
   - Verify GPIO 17 = Pin 22
   - Check 3.3V power and GND

3. **LED node not running:**
   ```bash
   # Start manually
   cd ~/dev/r2d2/ros2_ws && source install/setup.bash
   ros2 run r2d2_audio status_led_node
   ```

---

### Issue: PersonRegistry Not Working

**Symptoms:**
- System shows "target_person" instead of actual name
- "Person not resolved" warnings
- Models not auto-discovered

**Diagnosis:**
```bash
# 1. Check database exists
ls -la ~/dev/r2d2/data/persons.db

# 2. Check persons registered
cd ~/dev/r2d2/tests/face_recognition
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.list_persons())"
# Should list at least one person

# 3. Check model paths
python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); persons = r.list_persons(); [print(f'{p[\"display_name\"]}: face={p[\"face_model_path\"]}, gesture={p[\"gesture_model_path\"]}') for p in persons]"
```

**Solutions:**
1. **Database not initialized:**
   ```bash
   cd ~/dev/r2d2/tests/face_recognition
   python3 -c "from person_registry import PersonRegistry; PersonRegistry()"
   ```

2. **No persons registered:**
   ```bash
   # Auto-migrate from existing models
   python3 -c "from person_registry import PersonRegistry; r = PersonRegistry(); print(r.auto_migrate())"
   ```

3. **Model paths wrong:**
   - Retrain models (creates registry entries)
   - Or manually register using person_manager.py

---

## High CPU Usage

**Symptoms:**
- System sluggish
- High CPU usage by Python processes
- Reduced performance

**Diagnosis:**
```bash
# Check CPU usage
top -bn1 | grep python | head -10

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/perception/gesture_event

# Check frame skip settings
ros2 param get /image_listener recognition_frame_skip
ros2 param get /image_listener gesture_frame_skip
```

**Solutions:**
1. **Increase frame skip:**
   ```bash
   # Process fewer frames
   ros2 param set /image_listener recognition_frame_skip 3
   ros2 param set /image_listener gesture_frame_skip 7
   
   # Reduces CPU at cost of slower recognition
   ```

2. **Disable unused features:**
   ```bash
   # Disable gesture recognition if not needed
   ros2 param set /image_listener enable_gesture_recognition false
   ```

---

## Recovery Procedures

### Quick Service Restart

```bash
# Restart single service
sudo systemctl restart r2d2-audio-notification

# Restart all perception services
sudo systemctl restart r2d2-camera-perception \
  r2d2-audio-notification r2d2-gesture-intent
```

### Full System Reset

```bash
# Stop all services
sudo systemctl stop r2d2-gesture-intent \
  r2d2-audio-notification r2d2-camera-perception

# Wait 5 seconds
sleep 5

# Start in order
sudo systemctl start r2d2-audio-notification
sleep 2
sudo systemctl start r2d2-camera-perception
sleep 3
sudo systemctl start r2d2-gesture-intent

# Wait 10 seconds for initialization
sleep 10

# Verify all running
systemctl is-active r2d2-camera-perception \
  r2d2-audio-notification r2d2-gesture-intent
```

### Rebuild Packages

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

# Clean build (if needed)
rm -rf build install log

# Build perception packages
colcon build --packages-select r2d2_camera r2d2_perception \
  r2d2_audio r2d2_gesture r2d2_common

# Restart services
sudo systemctl restart r2d2-camera-perception \
  r2d2-audio-notification r2d2-gesture-intent
```

---

## Monitoring Commands

### Real-Time Monitoring

```bash
# Open multiple terminals:

# Terminal 1: Person ID
ros2 topic echo /r2d2/perception/person_id

# Terminal 2: Person status (color-coded)
ros2 topic echo /r2d2/audio/person_status --no-arr | \
  grep -oP '"status":\s*"\K\w+' --line-buffered | \
  while read status; do
    case $status in
      red)   echo -e "\033[1;31m🔴 RED\033[0m" ;;
      blue)  echo -e "\033[1;34m🔵 BLUE\033[0m" ;;
      green) echo -e "\033[1;32m🟢 GREEN\033[0m" ;;
    esac
  done

# Terminal 3: Gesture events
ros2 topic echo /r2d2/perception/gesture_event

# Terminal 4: Service logs
sudo journalctl -u r2d2-audio-notification -f | \
  grep --line-buffered -E "recognized|lost|Timer|RED"
```

### Performance Monitoring

```bash
# Check topic rates
ros2 topic hz /r2d2/perception/person_id      # ~6.5 Hz
ros2 topic hz /r2d2/audio/person_status       # ~10 Hz

# Check CPU
top -bn1 | grep -E "python|Cpu"

# Check memory
free -h
```

---

## Quick System Check Script

Save as `~/dev/r2d2/check_perception_system.sh`:

```bash
#!/bin/bash
echo "=== R2D2 Perception System Check ==="
echo ""

echo "Services:"
for service in r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent; do
  status=$(systemctl is-active $service)
  if [ "$status" = "active" ]; then
    echo "  ✅ $service"
  else
    echo "  ❌ $service: $status"
  fi
done

echo ""
echo "Nodes:"
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
nodes=$(ros2 node list | grep -E "audio|gesture|image|camera" | wc -l)
echo "  $nodes critical nodes found"

echo ""
echo "Hardware:"
if lsusb | grep -q Movidius; then
  echo "  ✅ Camera connected"
else
  echo "  ❌ Camera NOT FOUND"
fi

if [ "$OPENBLAS_CORETYPE" = "ARMV8" ]; then
  echo "  ✅ OPENBLAS_CORETYPE set"
else
  echo "  ❌ OPENBLAS_CORETYPE not set"
fi

if [ -d "/sys/class/gpio/gpio17" ]; then
  echo "  ✅ LED GPIO exported"
else
  echo "  ❌ LED GPIO not exported"
fi

echo ""
echo "Parameters:"
cd ~/dev/r2d2/ros2_ws && source install/setup.bash 2>/dev/null
echo "  red_timeout: $(ros2 param get /audio_notification_node red_status_timeout_seconds 2>&1 | grep -o '[0-9.]*')s"
echo "  audio_volume: $(ros2 param get /audio_notification_node audio_volume 2>&1 | grep -o '[0-9.]*')"

echo ""
echo "Test: Stand in camera and check person_id topic"
```

```bash
chmod +x ~/dev/r2d2/check_perception_system.sh
```

---

## Related Documentation

**Reference:**
- `100_PERCEPTION_STATUS_REFERENCE.md` - Technical reference

**Installation:**
- `101_PERCEPTION_STATUS_INSTALLATION.md` - Setup guide

**Quick Start:**
- `102_PERCEPTION_STATUS_QUICK_START.md` - Daily operations

**System:**
- `001_ARCHITECTURE_OVERVIEW.md` - Overall architecture
- `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` - Person registry

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Status:** Production troubleshooting guide

