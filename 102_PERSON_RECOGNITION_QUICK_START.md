# R2D2 Person Recognition System - Quick Start
## Fast Reference for Daily Use

**Last Updated:** December 17, 2025  
**Hardware:** OAK-D Lite Camera + PAM8403 Speaker + Optional RGB LED  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble

---

## Quick Launch

**Terminal 1: Camera + Perception + Recognition**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true
```

**Terminal 2: Audio Notifications (if not using service)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 3: LED Node (optional)**
```bash
ros2 run r2d2_audio status_led_node
```

**That's it!** System will auto-detect your face and play alerts.

---

## Monitor System

**Terminal 4 - Person Recognition:**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash
ros2 topic echo /r2d2/perception/person_id
```

**Terminal 5 - Status (RED/BLUE/GREEN):**
```bash
ros2 topic echo /r2d2/audio/person_status --no-arr
```

**Terminal 6 - Face Count:**
```bash
ros2 topic echo /r2d2/perception/face_count
```

---

## Test It

1. Launch camera + perception + recognition (Terminal 1)
2. Launch audio notifications (Terminal 2)
3. Monitor topics (Terminals 4-6)
4. **Stand in front of camera**
5. **Listen:** Should hear "Hello!" beep
6. **Watch:** Person ID shows "target_person"
7. **Walk away:** After ~20 seconds, hear "Oh, I lost you!"

---

## Common Commands

### Check Node Status
```bash
# List all nodes
ros2 node list

# Check specific nodes
ros2 node list | grep -E "camera|perception|audio"

# Expected:
# /camera_node
# /image_listener
# /audio_notification_node
```

### Check Topics
```bash
# List all topics
ros2 topic list | grep r2d2

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status

# Expected rates:
# person_id: ~6.5 Hz
# person_status: ~10 Hz
```

### Control Service
```bash
# Check service status
sudo systemctl status r2d2-audio-notification.service

# Start service
sudo systemctl start r2d2-audio-notification.service

# Stop service
sudo systemctl stop r2d2-audio-notification.service

# Restart service
sudo systemctl restart r2d2-audio-notification.service

# View logs
sudo journalctl -u r2d2-audio-notification.service -f
```

### Gesture Intent Service
```bash
# Check gesture intent status
sudo systemctl status r2d2-gesture-intent.service

# Start gesture service
sudo systemctl start r2d2-gesture-intent.service

# Stop gesture service
sudo systemctl stop r2d2-gesture-intent.service

# Restart gesture service
sudo systemctl restart r2d2-gesture-intent.service

# View gesture logs
sudo journalctl -u r2d2-gesture-intent.service -f
```

### Adjust Volume
```bash
# Increase volume (runtime)
ros2 param set /audio_notification_node audio_volume 0.3

# Volume levels:
# 0.05 = 5% (default, very quiet)
# 0.2 = 20% (moderate)
# 0.5 = 50% (loud)
# 1.0 = 100% (maximum)
```

### Change Parameters
```bash
# View all parameters
ros2 param list /audio_notification_node

# Change loss detection time
ros2 param set /audio_notification_node loss_confirmation_seconds 10.0

# Change jitter tolerance
ros2 param set /audio_notification_node jitter_tolerance_seconds 3.0

# View current value
ros2 param get /audio_notification_node audio_volume
```

---

## Training Commands

**Capture Training Data:**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 1_capture_training_data.py
```

**Train Model:**
```bash
python3 2_train_recognizer.py
```

**Test Model:**
```bash
python3 3_test_recognizer_demo.py
```

**Training Manager (Menu):**
```bash
python3 train_manager.py
# Options:
# 1. Train new person
# 2. Add more pictures
# 3. Retrain from existing
# 4. Test accuracy
# 5. Real-time test
# 6. List all models
# 7. Delete person
```

---

## View Status

**Live Stream Monitoring:**
```bash
cd ~/dev/r2d2
./tests/system/monitor_person_recognition.sh
```

**Full Pipeline Dashboard:**
```bash
cd ~/dev/r2d2
./tests/system/monitor_full_pipeline.sh
```

**Manual Status Check:**
```bash
# Watch person ID with timestamps
watch -n 0.5 'ros2 topic echo /r2d2/perception/person_id --once | grep data'

# Monitor status JSON
watch -n 0.5 'ros2 topic echo /r2d2/audio/person_status --once --no-arr'

# Check system health
watch -n 1 'ros2 node list && echo "---" && ros2 topic list | grep r2d2'
```

---

## Troubleshooting

### No Audio
```bash
# Check service
sudo systemctl status r2d2-audio-notification.service

# Test speaker directly
speaker-test -t wav -c 2 -D hw:1,0

# Increase volume
ros2 param set /audio_notification_node audio_volume 0.5

# Check ALSA
aplay -l | grep "card 1"
```

### Camera Not Working
```bash
# Check USB connection
lsusb | grep Movidius

# Test camera access
source ~/depthai_env/bin/activate
python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"

# Check environment
echo $OPENBLAS_CORETYPE
# Should show: ARMV8
```

### Always Returns "unknown"
```bash
# Lower threshold (accept weaker matches)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_confidence_threshold:=80.0

# Or retrain with more images
cd ~/dev/r2d2/tests/face_recognition
python3 train_manager.py  # Select option 2
```

### High CPU Usage
```bash
# Increase frame skip (reduce CPU)
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3

# Check CPU usage
top -bn1 | grep python
```

### Rebuild System
```bash
cd ~/dev/r2d2/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Performance Metrics

**Expected Values:**

| Metric | Value |
|--------|-------|
| **Camera FPS** | 30 Hz |
| **Face Detection Rate** | 13 Hz |
| **Person ID Rate** | 6.5 Hz (with skip=2) |
| **Status Rate** | 10 Hz |
| **CPU Usage** | 15-25% total |
| **Memory** | ~500 MB |
| **Recognition Latency** | <100 ms |
| **Loss Detection Time** | ~20s (5s jitter + 15s confirm) |

**Warning Thresholds:**
- Camera < 25 Hz
- Detection < 10 Hz
- Person ID < 5 Hz
- CPU > 50%
- Memory > 2 GB

---

## Hardware Info

### OAK-D Lite Camera
**Connection:** USB 3.0 (direct, not through hub)  
**Resolution:** 1920×1080 @ 30 FPS  
**Interface:** DepthAI SDK 2.31.0.0  
**Auto-Detection:** Via `lsusb | grep Movidius`

### PAM8403 Speaker
**Connection:** J511 Audio Header (Pin 9: HPO_L, Pin 2: AGND)  
**Power:** Jetson Pin 2 (5V), Pin 6 (GND)  
**Sample Rate:** 44.1 kHz (ALSA Card 1, Device 0)  
**Volume Control:** Via ROS parameter `audio_volume`

### RGB LED (Optional)
**GPIO Pins:**
- RED: GPIO 17 (Pin 11)
- GREEN: GPIO 27 (Pin 13)
- BLUE: GPIO 22 (Pin 15)
- GND: Pin 14

**Status Colors:**
- 🔴 RED = Target person recognized
- 🔵 BLUE = No person (idle)
- 🟢 GREEN = Unknown person detected

---

## Configuration Files

### Environment Setup
**Location:** `~/.bashrc`
```bash
export OPENBLAS_CORETYPE=ARMV8
```

### ALSA Audio
**Location:** `/etc/asound.conf`
```
pcm.!default {
    type asym
    playback.pcm "speaker_out"
}

pcm.speaker_out {
    type dmix
    slave {
        pcm "hw:1,0"
        rate 44100
        channels 2
    }
}
```

### ROS 2 Parameters
**View current:**
```bash
ros2 param list /audio_notification_node
ros2 param get /audio_notification_node audio_volume
```

**Set runtime:**
```bash
ros2 param set /audio_notification_node audio_volume 0.3
```

**Set permanent (edit service):**
```bash
sudo nano /etc/systemd/system/r2d2-audio-notification.service
# Add to ExecStart: audio_volume:=0.3
sudo systemctl daemon-reload
sudo systemctl restart r2d2-audio-notification.service
```

---

## File Locations

**Training Data:**
- Images: `~/dev/r2d2/data/face_recognition/{person}/`
- Models: `~/dev/r2d2/data/face_recognition/models/{person}_lbph.xml`
- Registry: `~/dev/r2d2/data/persons.db` (auto-resolution database)

**Audio Files:**
- Source: `~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/`
- Installed: `~/dev/r2d2/ros2_ws/install/r2d2_audio/share/r2d2_audio/assets/audio/`

**Service Files:**
- Service: `/etc/systemd/system/r2d2-audio-notification.service`
- Startup script: `/home/severin/dev/r2d2/start_audio_service.sh`

**ROS 2 Packages:**
- Workspace: `~/dev/r2d2/ros2_ws/`
- Source: `~/dev/r2d2/ros2_ws/src/`
- Install: `~/dev/r2d2/ros2_ws/install/`

**Training Scripts:**
- Location: `~/dev/r2d2/tests/face_recognition/`
- Scripts: `1_capture_training_data.py`, `2_train_recognizer.py`, `3_test_recognizer_demo.py`

---

## Launch Options

### Standard Launch
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true
```

### Custom Threshold (More Detections)
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_confidence_threshold:=80.0
```

### Minimal CPU (Higher Frame Skip)
```bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  recognition_frame_skip:=3
```

### Loud, Responsive Audio
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.8 \
  loss_confirmation_seconds:=10.0
```

### Quiet, Patient Audio
```bash
ros2 launch r2d2_audio audio_notification.launch.py \
  audio_volume:=0.2 \
  loss_confirmation_seconds:=20.0
```

---

## Integration with R2D2 System

### Architecture
```
OAK-D Camera
    ↓
Camera Node (ROS 2)
    ↓
Perception Pipeline
    ↓
Face Recognition
    ↓
Audio Notification (State Machine)
    ↓
Audio Playback + LED Display
```

### Subscribe to Topics (Example)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PersonListener(Node):
    def __init__(self):
        super().__init__('person_listener')
        self.subscription = self.create_subscription(
            String,
            '/r2d2/perception/person_id',
            self.callback,
            10)
    
    def callback(self, msg):
        self.get_logger().info(f'Person detected: {msg.data}')

def main():
    rclpy.init()
    node = PersonListener()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## Tips & Best Practices

### For Best Recognition
- Train with 80+ diverse images
- Vary lighting conditions during training
- Include different angles and expressions
- Position camera 1-3 meters from person
- Avoid backlighting

### For Natural Behavior
- Default timing (20s loss) works well for most use cases
- Increase `jitter_tolerance_seconds` (e.g., 7.0) for noisy environments
- Decrease `loss_confirmation_seconds` (e.g., 10.0) for faster response

### For System Performance
- Use frame_skip=2 for balanced CPU/accuracy
- Increase to frame_skip=3 if CPU is constrained
- Monitor with: `top -bn1 | grep python`
- Keep network latency low (if using remote nodes)

### For Debugging
- Check logs: `sudo journalctl -u r2d2-audio-notification.service -f`
- Monitor topics: `ros2 topic hz <topic>`
- Check node status: `ros2 node list`
- View parameters: `ros2 param list <node>`

---

## Aliases for Convenience

Add to `~/.bashrc`:
```bash
# R2D2 Person Recognition aliases
alias r2d2-person='cd ~/dev/r2d2/ros2_ws && source ~/depthai_env/bin/activate && source ~/.bashrc && source install/setup.bash && ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true'

alias r2d2-audio='cd ~/dev/r2d2/ros2_ws && source install/setup.bash && ros2 launch r2d2_audio audio_notification.launch.py'

alias r2d2-status='ros2 topic echo /r2d2/audio/person_status --no-arr'

alias r2d2-person-id='ros2 topic echo /r2d2/perception/person_id'

alias r2d2-service='sudo systemctl status r2d2-audio-notification.service'

alias r2d2-logs='sudo journalctl -u r2d2-audio-notification.service -f'
```

Then use:
```bash
r2d2-person         # Launch camera + perception + recognition
r2d2-audio          # Launch audio notifications
r2d2-status         # Watch status messages
r2d2-person-id      # Watch person recognition
r2d2-service        # Check service status
r2d2-logs           # View service logs
```

---

## Full Documentation

- **Reference:** [100_PERSON_RECOGNITION_REFERENCE.md](100_PERSON_RECOGNITION_REFERENCE.md) - Complete architecture
- **Installation:** [101_PERSON_RECOGNITION_INSTALLATION.md](101_PERSON_RECOGNITION_INSTALLATION.md) - Setup guide
- **Architecture:** [001_ARCHITECTURE_OVERVIEW.md](001_ARCHITECTURE_OVERVIEW.md) - System overview

---

## Support Resources

- **ROS 2 Humble:** https://docs.ros.org/en/humble/
- **DepthAI SDK:** https://docs.luxonis.com/software-v3/depthai/
- **OAK-D Lite:** https://docs.luxonis.com/hardware/products/OAK-D-Lite/
- **OpenCV Face Recognition:** https://docs.opencv.org/4.x/df/d25/classcv_1_1face_1_1LBPHFaceRecognizer.html

---

🎉 **Happy Recognizing!**

---

**Quick Start Version:** 1.0  
**Last Updated:** December 17, 2025  
**Hardware:** OAK-D Lite Camera + PAM8403 Speaker  
**Status:** Production-ready


