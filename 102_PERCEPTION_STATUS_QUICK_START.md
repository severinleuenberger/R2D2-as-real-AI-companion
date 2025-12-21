# R2D2 Perception and Status System - Quick Start
## Fast Reference for Daily Operations

**Last Updated:** December 21, 2025  
**Hardware:** OAK-D Lite + PAM8403 Speaker + White LED  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Quick Launch (Manual Mode)

**Terminal 1: Camera + Perception**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  enable_gesture_recognition:=true
```

**Terminal 2: Audio + LED + Logging**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 3: Gesture Intent (Optional)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_gesture gesture_intent.launch.py
```

**That's it!** System will auto-detect your face, play alerts, and respond to gestures.

---

## Quick Status Check (Auto-Start Mode)

```bash
# Check all services
sudo systemctl is-active r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent
# EXPECT: active active active

# Check all nodes
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 node list | grep -E "audio|gesture|image|camera"
# EXPECT: 5+ nodes listed
```

---

## Quick Test Procedure

1. **Stand in front of camera**
   - LED should turn ON
   - Listen for "Hello!" beep (2% volume, very quiet)
   - Status: RED (recognized)

2. **Make index finger up gesture** ☝️
   - Only works when LED is ON (RED status)
   - Listen for gesture start beep
   - Gesture event published

3. **Make fist gesture** ✊
   - Listen for gesture stop beep
   - Gesture event published

4. **Walk away from camera**
   - Wait 15-20 seconds
   - LED turns OFF
   - Listen for "Lost you!" beep
   - Status: BLUE (lost)

---

## Quick Topic Monitoring

**Person Recognition:**
```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash
ros2 topic echo /r2d2/perception/person_id --once
# EXPECT: "target_person" (when in front of camera)
```

**Person Status (RED/GREEN/BLUE):**
```bash
ros2 topic echo /r2d2/audio/person_status --once
# EXPECT: {"status": "red",...} when recognized
```

**Gesture Events:**
```bash
ros2 topic echo /r2d2/perception/gesture_event
# Make gesture, EXPECT: "index_finger_up" or "fist"
```

**Face Count:**
```bash
ros2 topic echo /r2d2/perception/face_count --once
# EXPECT: 1 when face detected, 0 when no face
```

---

## Service Management

### Check Status

```bash
# Individual services
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-audio-notification.service
sudo systemctl status r2d2-gesture-intent.service

# All at once
systemctl is-active r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent
```

### Start/Stop/Restart

```bash
# Start all services
sudo systemctl start r2d2-audio-notification.service
sleep 2
sudo systemctl start r2d2-camera-perception.service
sleep 3
sudo systemctl start r2d2-gesture-intent.service

# Restart single service
sudo systemctl restart r2d2-audio-notification.service

# Restart all services
sudo systemctl restart r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent

# Stop all services
sudo systemctl stop r2d2-gesture-intent.service
sudo systemctl stop r2d2-camera-perception.service
sudo systemctl stop r2d2-audio-notification.service
```

### View Logs

```bash
# Audio notification logs (status transitions)
sudo journalctl -u r2d2-audio-notification.service -f

# Gesture intent logs (gesture actions)
sudo journalctl -u r2d2-gesture-intent.service -f

# Camera perception logs
sudo journalctl -u r2d2-camera-perception.service -f

# All logs combined
sudo journalctl -f -u r2d2-camera-perception -u r2d2-audio-notification -u r2d2-gesture-intent
```

---

## Quick Parameter Adjustments

### Audio Volume (Runtime)

```bash
# Current volume
ros2 param get /audio_notification_node audio_volume
# Default: 0.02 (2%, very quiet)

# Increase volume temporarily
ros2 param set /audio_notification_node audio_volume 0.5
# 0.05 = 5% (quiet)
# 0.2 = 20% (moderate)
# 0.5 = 50% (loud)
# 1.0 = 100% (maximum)
```

### Status Timers

```bash
# View all parameters
ros2 param list /audio_notification_node

# Change RED timeout (how long to wait before loss)
ros2 param set /audio_notification_node red_status_timeout_seconds 20.0

# View current value
ros2 param get /audio_notification_node red_status_timeout_seconds
```

### Permanent Changes

**Edit config file:**
```bash
nano ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml
# Change audio_volume: 0.02 to desired value
```

**Rebuild and restart:**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
sudo systemctl restart r2d2-audio-notification.service
```

---

## Training Commands

### Train New Person

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 train_manager.py
# Select [1] Train new person
# Enter person name (e.g., "severin")
# Follow prompts to capture face images
# Then select [8] Train gestures for person
# Use SAME person name
```

### Add More Training Images

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 train_manager.py
# Select [2] Add additional pictures
# Select [10] Train gesture model from existing images
```

### Test Models

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 train_manager.py
# Select [5] Test recognizer (real-time)
# Select [11] Test gesture classifier
```

---

## Performance Monitoring

### Check Topic Rates

```bash
cd ~/dev/r2d2/ros2_ws && source install/setup.bash

# Face recognition rate (should be ~6.5 Hz)
ros2 topic hz /r2d2/perception/person_id

# Status rate (should be ~10 Hz)
ros2 topic hz /r2d2/audio/person_status

# Gesture events (event-driven, no fixed rate)
ros2 topic hz /r2d2/perception/gesture_event
```

### Check CPU Usage

```bash
# All Python processes
top -bn1 | grep python | head -5

# Specific nodes
ps aux | grep -E "image_listener|audio_notification|gesture_intent"
```

### Live Monitoring

**Color-Coded Person ID:**
```bash
ros2 topic echo /r2d2/perception/person_id --no-arr | grep -oP "data: '\K[^']+" --line-buffered | while read id; do
  case $id in
    severin|target_person) echo -e "\033[1;32m✅ $id recognized\033[0m" ;;
    unknown)   echo -e "\033[1;33m❓ Unknown person\033[0m" ;;
    no_person) echo -e "\033[1;90m👤 No person\033[0m" ;;
  esac
done
```

**Color-Coded Status:**
```bash
ros2 topic echo /r2d2/audio/person_status --no-arr | grep -oP '"status":\s*"\K\w+' --line-buffered | while read status; do
  case $status in
    red)   echo -e "\033[1;31m🔴 RED - Recognized\033[0m" ;;
    blue)  echo -e "\033[1;34m🔵 BLUE - No person\033[0m" ;;
    green) echo -e "\033[1;32m🟢 GREEN - Unknown\033[0m" ;;
  esac
done
```

**Gesture Events:**
```bash
ros2 topic echo /r2d2/perception/gesture_event --no-arr | grep -oP "data: '\K[^']+" --line-buffered | while read gesture; do
  case $gesture in
    index_finger_up) echo -e "\033[1;36m☝️  INDEX FINGER UP\033[0m" ;;
    fist)            echo -e "\033[1;35m✊ FIST\033[0m" ;;
  esac
done
```

---

## Quick Troubleshooting

### No Audio Heard

```bash
# Check volume
ros2 param get /audio_notification_node audio_volume
# If too low (< 0.1), increase temporarily:
ros2 param set /audio_notification_node audio_volume 0.5

# Test speaker directly
ffplay -nodisp -autoexit -af "volume=0.50" \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# Check ALSA device
aplay -l | grep "card 1"
```

### Camera Not Working

```bash
# Check USB connection
lsusb | grep Movidius
# EXPECT: 03e7:2485 Intel Movidius MyriadX

# Check environment variable
echo $OPENBLAS_CORETYPE
# EXPECT: ARMV8

# Test camera in virtualenv
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"
# EXPECT: 1
```

### Always Returns "unknown"

```bash
# Option 1: Lower threshold (accept weaker matches)
ros2 param set /image_listener recognition_confidence_threshold 80.0

# Option 2: Add more training images
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py  # Select option 2
```

### Gestures Not Working

```bash
# Check person status is RED
ros2 topic echo /r2d2/audio/person_status --once | grep status
# EXPECT: "red"

# Check gestures are detected
timeout 10 ros2 topic echo /r2d2/perception/gesture_event
# Make gesture, should see event

# Check gesture intent logs
sudo journalctl -u r2d2-gesture-intent --since "1 minute ago" | grep "ignored"
# Look for blocking reasons (cooldown, not RED, etc.)
```

### LED Not Working

```bash
# Check GPIO permissions
ls -la /sys/class/gpio/gpio17/
# Should be accessible by user "severin"

# Check LED node is running
ros2 node list | grep status_led
# EXPECT: /status_led_node

# Manually test GPIO
echo 1 > /sys/class/gpio/gpio17/value  # ON
echo 0 > /sys/class/gpio/gpio17/value  # OFF
```

### High CPU Usage

```bash
# Increase frame skip (reduce CPU)
ros2 param set /image_listener recognition_frame_skip 3
ros2 param set /image_listener gesture_frame_skip 7

# Check current skip values
ros2 param get /image_listener recognition_frame_skip
ros2 param get /image_listener gesture_frame_skip
```

---

## Rebuild After Code Changes

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash

# Rebuild specific packages
colcon build --packages-select r2d2_audio r2d2_gesture r2d2_perception

# Rebuild all
colcon build

# Restart services
sudo systemctl restart r2d2-camera-perception r2d2-audio-notification r2d2-gesture-intent
```

---

## Performance Benchmarks

**Expected Values:**

| Metric | Value | Notes |
|--------|-------|-------|
| Camera FPS | 30 Hz | Native rate |
| Face detection | 13 Hz | Downscaled processing |
| Face recognition | 6.5 Hz | With frame_skip=2 |
| Gesture recognition | ~6 Hz | With frame_skip=5 |
| Status publishing | 10 Hz | Audio notification node |
| RED response time | ~460ms | Rolling window (3 in 1.0s) |
| Total CPU | 15-25% | All components |
| Memory | ~500 MB | All components |

---

## Quick Reference: Topic List

**Published by Perception (`image_listener`):**
- `/r2d2/perception/brightness` (Float32, 13 Hz)
- `/r2d2/perception/face_count` (Int32, 13 Hz)
- `/r2d2/perception/person_id` (String, 6.5 Hz)
- `/r2d2/perception/face_confidence` (Float32, 6.5 Hz)
- `/r2d2/perception/is_target_person` (Bool, 6.5 Hz)
- `/r2d2/perception/gesture_event` (String, event-driven)

**Published by Audio (`audio_notification_node`):**
- `/r2d2/audio/person_status` (String JSON, 10 Hz)
- `/r2d2/audio/notification_event` (String, event-driven)

---

## Quick Reference: State Machine

**🔴 RED (Recognized)**
- Entry: 3 recognition matches in 1.0s window
- LED: ON
- Audio: "Hello!" beep (2% volume)
- Timer: 15s (resets on each match)
- Exit: Timer expires → GREEN or BLUE

**🟢 GREEN (Unknown Person)**
- Entry: Face detected for 2s, not recognized
- LED: OFF
- Audio: Silent
- Exit: Target appears → RED, No face 3s → BLUE

**🔵 BLUE (No Person)**
- Entry: No face for 5s + RED timer expired
- LED: OFF
- Audio: "Lost you!" beep from RED
- Exit: Target appears → RED, Unknown face 2s → GREEN

---

## Next Steps

**For more details:**
- `100_PERCEPTION_STATUS_REFERENCE.md` - Complete technical reference
- `101_PERCEPTION_STATUS_INSTALLATION.md` - Full installation guide
- `103_PERCEPTION_STATUS_TROUBLESHOOTING.md` - Detailed troubleshooting

**Related systems:**
- `200_SPEECH_SYSTEM_REFERENCE.md` - Speech integration
- `250_PERSON_MANAGEMENT_SYSTEM_REFERENCE.md` - Person registry

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Status:** Production ready quick reference

