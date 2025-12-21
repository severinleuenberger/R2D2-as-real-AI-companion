# R2D2 Perception and Status System - Installation Guide
## Complete Setup: Face Recognition, Gestures, and Status Management

**Date:** December 21, 2025  
**Status:** Complete Installation Guide  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Estimated Time:** 2-3 hours

---

## Prerequisites

### Hardware Requirements

✅ **NVIDIA Jetson AGX Orin 64GB**
- JetPack 6.x installed
- At least 10GB free storage
- USB 3.0 ports available

✅ **OAK-D Lite Camera**
- USB 3.0 USB-C to USB-A cable
- Auto-focus version recommended

✅ **Speaker + PAM8403 Amplifier**
- 8Ω 3W speaker
- PAM8403 Class-D amplifier board
- Jumper wires

✅ **White LED Panel**
- Non-addressable white LED array (16 SMD LEDs)
- 3V DC, 20-50mA
- Jumper wires

### Software Requirements

✅ **ROS 2 Humble** installed and working
```bash
ros2 --version
# Should show: ROS 2 Humble
```

✅ **Python 3.10**
```bash
python3 --version
# Should show: Python 3.10.x
```

✅ **Git**
```bash
git --version
```

---

## Part 1: Hardware Setup (30 minutes)

### Step 1.1: Connect OAK-D Lite Camera

**Physical Connection:**
1. Plug OAK-D Lite into USB 3.0 port on Jetson
2. Use direct USB port (not through hub)
3. Blue USB connector = USB 3.0

**Verify Detection:**
```bash
lsusb | grep Movidius
# Expected: Bus 001 Device 003: ID 03e7:2485 Intel Movidius MyriadX
```

If not detected: Try different USB port, check cable quality

### Step 1.2: Wire PAM8403 Amplifier

**Complete Wiring:**
```
Jetson 40-pin header (J30):
  Pin 2  (5V)   → PAM8403 +5V
  Pin 6  (GND)  → PAM8403 GND

Jetson audio panel header (J511):
  Pin 9  (HPO_L) → PAM8403 LIN
  Pin 2  (AGND)  → PAM8403 GND

PAM8403 output:
  L+ → speaker + (red wire)
  L− → speaker − (black wire)
```

**Verification:**
```bash
aplay -l
# Should show Card 1: NVIDIA Jetson AGX Orin APE
```

### Step 1.3: Wire White LED

**GPIO Connection:**
```
Jetson 40-pin header:
  Red wire   → Pin 1 or 17 (3.3V power)
  Blue wire  → Pin 22 (GPIO 17 signal)
  Black wire → Pin 6 (GND)
```

**For detailed wiring, see:** `HARDWARE_WHITE_LED_WIRING.md`

---

## Part 2: Software Installation (40 minutes)

### Step 2.1: Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-dev python3-pip python3-venv \
  libjpeg-dev libpng-dev libopenexr-dev \
  libtbb-dev libatlas-base-dev libgl1-mesa-glx \
  portaudio19-dev libasound2-dev alsa-utils \
  pulseaudio-utils ffmpeg

# Takes ~5-10 minutes
```

### Step 2.2: Create DepthAI Virtual Environment

```bash
cd ~
python3 -m venv depthai_env
source ~/depthai_env/bin/activate

# You should see: (depthai_env) in your prompt
```

**Important:** Activate this virtualenv before using camera/training tools.

### Step 2.3: Install DepthAI SDK

```bash
# Ensure virtualenv active
source ~/depthai_env/bin/activate

# Install DepthAI
pip install --upgrade pip
pip install depthai==2.31.0.0

# Verify installation
python3 -c "import depthai as dai; print(f'DepthAI {dai.__version__}')"
# Expected: DepthAI 2.31.0.0
```

### Step 2.4: Install OpenCV with Contrib

**Install OpenCV (LBPH Face Recognition):**
```bash
# Still in depthai_env
pip install opencv-python opencv-contrib-python

# Verify LBPH is available
python3 -c "import cv2; print('LBPH Available:', hasattr(cv2.face, 'LBPHFaceRecognizer_create'))"
# Expected: LBPH Available: True
```

### Step 2.5: Install MediaPipe and scikit-learn (Gesture Recognition)

```bash
# Still in depthai_env
pip install mediapipe scikit-learn

# Verify MediaPipe
python3 -c "import mediapipe as mp; print(f'MediaPipe {mp.__version__}')"

# Verify scikit-learn
python3 -c "import sklearn; print(f'scikit-learn {sklearn.__version__}')"
```

### Step 2.6: Set Critical Environment Variable

```bash
# Add to ~/.bashrc for persistence
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc

# Verify it's set
echo $OPENBLAS_CORETYPE
# Expected: ARMV8
```

**Critical:** This prevents "Illegal instruction" errors on ARM64.

---

## Part 3: ROS 2 Workspace Setup (30 minutes)

### Step 3.1: Create or Update Workspace

```bash
# If workspace doesn't exist
mkdir -p ~/dev/r2d2/ros2_ws/src
cd ~/dev/r2d2/ros2_ws

# If workspace exists, just navigate
cd ~/dev/r2d2/ros2_ws
```

### Step 3.2: Create ROS 2 Packages

**Packages to create (if not exist):**
- `r2d2_camera` - Camera driver
- `r2d2_perception` - Image processing, face recognition, gesture recognition
- `r2d2_audio` - Audio notifications, LED, logging
- `r2d2_gesture` - Gesture intent control
- `r2d2_bringup` - Launch files
- `r2d2_common` - Shared utilities (PersonConfig)

**Create skeleton structure:**
```bash
cd ~/dev/r2d2/ros2_ws/src

# If packages don't exist, create them
ros2 pkg create r2d2_camera --build-type ament_python --dependencies rclpy sensor_msgs std_msgs
ros2 pkg create r2d2_perception --build-type ament_python --dependencies rclpy sensor_msgs std_msgs cv_bridge
ros2 pkg create r2d2_audio --build-type ament_python --dependencies rclpy std_msgs
ros2 pkg create r2d2_gesture --build-type ament_python --dependencies rclpy std_msgs std_srvs
ros2 pkg create r2d2_bringup --build-type ament_python
ros2 pkg create r2d2_common --build-type ament_python
```

### Step 3.3: Install ROS 2 Node Files

**Copy or verify node files exist in correct locations:**

```
ros2_ws/src/
├── r2d2_camera/r2d2_camera/
│   └── oak_camera_node.py
├── r2d2_perception/r2d2_perception/
│   └── image_listener.py
├── r2d2_audio/r2d2_audio/
│   ├── audio_notification_node.py
│   ├── status_led_node.py
│   └── database_logger_node.py
├── r2d2_gesture/r2d2_gesture/
│   └── gesture_intent_node.py
├── r2d2_common/r2d2_common/
│   ├── person_config.py
│   └── person_registry.py
└── r2d2_bringup/launch/
    ├── r2d2_camera_perception.launch.py
    └── gesture_intent.launch.py
```

### Step 3.4: Install Audio Assets

```bash
# Create audio assets directory
mkdir -p ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio

# Copy R2D2 sound files (if you have them)
# Voicy_R2-D2 - 2.mp3  (recognition "Hello!")
# Voicy_R2-D2 - 5.mp3  (loss "Lost you!")
# Voicy_R2-D2 - 16.mp3 (gesture start)
# Voicy_R2-D2 - 20.mp3 (gesture stop)
```

### Step 3.5: Create Configuration Files

**Audio Parameters:**
```bash
mkdir -p ~/dev/r2d2/ros2_ws/src/r2d2_audio/config

cat > ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml << 'EOF'
/**:
  ros__parameters:
    # RED-first architecture parameters
    red_entry_match_threshold: 3
    red_entry_window_seconds: 1.0
    
    # Status timers
    red_status_timeout_seconds: 15.0
    green_entry_delay: 2.0
    blue_entry_delay: 3.0
    
    # Audio
    audio_volume: 0.02  # 2% volume (very quiet)
    cooldown_seconds: 2.0
    recognition_cooldown_after_loss_seconds: 5.0
    
    # Audio files
    recognition_audio_file: "Voicy_R2-D2 - 2.mp3"
    loss_audio_file: "Voicy_R2-D2 - 5.mp3"
    
    # Target person (auto-resolved from PersonRegistry)
    target_person: "target_person"
EOF
```

**Gesture Parameters:**
```bash
mkdir -p ~/dev/r2d2/ros2_ws/src/r2d2_gesture/config

cat > ~/dev/r2d2/ros2_ws/src/r2d2_gesture/config/gesture_params.yaml << 'EOF'
/**:
  ros__parameters:
    enabled: true
    cooldown_start_seconds: 5.0
    cooldown_stop_seconds: 3.0
    auto_shutdown_enabled: true
    auto_shutdown_timeout_seconds: 35.0
    audio_feedback_enabled: true
    audio_volume: 0.02  # 2% volume (very quiet)
EOF
```

### Step 3.6: Build Workspace

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
export OPENBLAS_CORETYPE=ARMV8

# Build all packages
colcon build --packages-select \
  r2d2_camera \
  r2d2_perception \
  r2d2_audio \
  r2d2_gesture \
  r2d2_common \
  r2d2_bringup

# Source the workspace
source install/setup.bash

# Takes ~2-5 minutes
```

---

## Part 4: Audio Configuration (15 minutes)

### Step 4.1: Configure ALSA

**Create/Edit `/etc/asound.conf`:**
```bash
sudo nano /etc/asound.conf
```

**Add content:**
```
pcm.!default {
    type asym
    playback.pcm "speaker_out"
}

pcm.speaker_out {
    type dmix
    ipc_key 1234
    slave {
        pcm "hw:1,0"
        rate 44100
        channels 2
        period_size 4096
        buffer_size 65536
    }
}
```

**Save and exit:** Ctrl+X, Y, Enter

### Step 4.2: Test Audio

```bash
# Test direct playback
ffplay -nodisp -autoexit -af "volume=0.30" \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# Should hear R2D2 beep at 30% volume
```

---

## Part 5: Training System Setup (15 minutes)

### Step 5.1: Create Data Directories

```bash
mkdir -p ~/dev/r2d2/data/face_recognition/models
mkdir -p ~/dev/r2d2/data/gesture_recognition/models
mkdir -p ~/dev/r2d2/tests/face_recognition
```

### Step 5.2: Install Training Scripts

**Verify training scripts exist in:**
```
~/dev/r2d2/tests/face_recognition/
├── train_manager.py
├── _face_capture_module.py
├── _face_train_module.py
├── _face_test_module.py
├── _gesture_capture_module.py
├── _gesture_train_module.py
├── _gesture_test_module.py
├── person_manager.py
└── person_registry.py
```

### Step 5.3: Initialize Person Registry Database

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# Initialize database
python3 -c "from person_registry import PersonRegistry; PersonRegistry()"

# Verify database created
ls -la ~/dev/r2d2/data/persons.db
```

---

## Part 6: Training (45 minutes)

### Step 6.1: Train Face Recognition

```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

python3 train_manager.py
# Select [1] Train new person
# Enter person name (e.g., "target_person")
# Stand in front of camera, follow prompts
# Move head slowly during capture (10-15 seconds)
# Training takes 2-3 minutes

# Result: ~/dev/r2d2/data/face_recognition/models/target_person_lbph.xml
```

### Step 6.2: Train Gesture Recognition

```bash
# Same session or new session
python3 train_manager.py
# Select [8] Train gestures for person
# Use SAME person name as face recognition
# Capture index finger up (15 seconds)
# Capture fist (15 seconds)
# Training takes 2-3 minutes
# Test model (30 seconds)

# Result: ~/dev/r2d2/data/gesture_recognition/models/target_person_gesture_classifier.pkl
```

**Important:** Use same person name for face and gesture training!

---

## Part 7: System Services (20 minutes)

### Step 7.1: Create Service Files

**Camera-Perception Service:**
```bash
sudo nano /etc/systemd/system/r2d2-camera-perception.service
```

```ini
[Unit]
Description=R2D2 Camera Perception Service
After=network.target
Wants=r2d2-audio-notification.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/home/severin/dev/r2d2/start_camera_perception.sh
ExecStop=/usr/bin/pkill -f "ros2 launch.*r2d2_camera_perception"
User=severin
WorkingDirectory=/home/severin/dev/r2d2
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

**Audio-Notification Service:**
```bash
sudo nano /etc/systemd/system/r2d2-audio-notification.service
```

```ini
[Unit]
Description=R2D2 Audio Notification Service
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/home/severin/dev/r2d2/start_audio_service.sh
ExecStop=/usr/bin/pkill -f "ros2 launch.*audio_notification"
User=severin
WorkingDirectory=/home/severin/dev/r2d2
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

**Gesture-Intent Service:**
```bash
sudo nano /etc/systemd/system/r2d2-gesture-intent.service
```

```ini
[Unit]
Description=R2D2 Gesture Intent Service
After=network.target r2d2-camera-perception.service
Requires=r2d2-camera-perception.service

[Service]
Type=exec
ExecStart=/home/severin/dev/r2d2/start_gesture_intent.sh
User=severin
WorkingDirectory=/home/severin/dev/r2d2
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

### Step 7.2: Create Startup Scripts

**Camera-Perception Startup:**
```bash
nano ~/dev/r2d2/start_camera_perception.sh
```

```bash
#!/bin/bash
cd /home/severin/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_CORETYPE=ARMV8

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true \
  enable_gesture_recognition:=true &

# Store PID for service management
echo $! > /tmp/r2d2_camera_perception.pid
wait
```

```bash
chmod +x ~/dev/r2d2/start_camera_perception.sh
```

**Audio-Notification Startup:**
```bash
nano ~/dev/r2d2/start_audio_service.sh
```

```bash
#!/bin/bash
cd /home/severin/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_CORETYPE=ARMV8

ros2 launch r2d2_audio audio_notification.launch.py &
echo $! > /tmp/r2d2_audio_notification.pid
wait
```

```bash
chmod +x ~/dev/r2d2/start_audio_service.sh
```

**Gesture-Intent Startup:**
```bash
nano ~/dev/r2d2/start_gesture_intent.sh
```

```bash
#!/bin/bash
cd /home/severin/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_CORETYPE=ARMV8

ros2 launch r2d2_gesture gesture_intent.launch.py
```

```bash
chmod +x ~/dev/r2d2/start_gesture_intent.sh
```

### Step 7.3: Enable and Start Services

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start
sudo systemctl enable r2d2-camera-perception.service
sudo systemctl enable r2d2-audio-notification.service
sudo systemctl enable r2d2-gesture-intent.service

# Start services
sudo systemctl start r2d2-audio-notification.service
sleep 2
sudo systemctl start r2d2-camera-perception.service
sleep 3
sudo systemctl start r2d2-gesture-intent.service

# Check status
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-audio-notification.service
sudo systemctl status r2d2-gesture-intent.service
```

---

## Part 8: Verification (15 minutes)

### Step 8.1: Check Services Running

```bash
# All should show "active (running)"
sudo systemctl status r2d2-camera-perception.service
sudo systemctl status r2d2-audio-notification.service
sudo systemctl status r2d2-gesture-intent.service
```

### Step 8.2: Monitor Topics

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash

# Check person recognition
ros2 topic echo /r2d2/perception/person_id --once
# Stand in front of camera, should show your person name

# Check person status
ros2 topic echo /r2d2/audio/person_status --once
# Should show RED status when recognized

# Check gesture events
ros2 topic echo /r2d2/perception/gesture_event
# Make gesture (index finger up or fist), should show gesture name
```

### Step 8.3: Verify Audio and LED

**Audio:**
- Stand in front of camera
- Wait for "Hello!" beep (2% volume, very quiet)
- Walk away, wait 20 seconds
- Wait for "Lost you!" beep

**LED:**
- Should turn ON when you're recognized (RED status)
- Should turn OFF when you walk away (BLUE status)

### Step 8.4: Test Gestures

**Prerequisites:**
- Stand in front of camera
- LED should be ON (RED status)

**Test:**
1. Raise index finger (pointing up)
   - Should trigger gesture event
2. Make fist (all fingers closed)
   - Should trigger gesture event

---

## Troubleshooting

### Issue: Camera not detected

```bash
lsusb | grep Movidius
# If not found, try different USB port or cable
```

### Issue: "Illegal instruction" error

```bash
# Ensure OPENBLAS_CORETYPE is set
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc
```

### Issue: No audio heard

```bash
# Test audio directly
ffplay -nodisp -autoexit -af "volume=0.50" \
  ~/dev/r2d2/ros2_ws/src/r2d2_audio/r2d2_audio/assets/audio/Voicy_R2-D2\ -\ 2.mp3

# If still quiet, check volume in config:
cat ~/dev/r2d2/ros2_ws/src/r2d2_audio/config/audio_params.yaml | grep audio_volume
# Should be 0.02 (2%)
```

### Issue: Services won't start

```bash
# Check logs
sudo journalctl -u r2d2-camera-perception.service -n 50
sudo journalctl -u r2d2-audio-notification.service -n 50
sudo journalctl -u r2d2-gesture-intent.service -n 50
```

---

## Next Steps

1. **Verify system works after reboot:**
   ```bash
   sudo reboot
   # Wait 60 seconds, test recognition and gestures
   ```

2. **Read operational documentation:**
   - `102_PERCEPTION_STATUS_QUICK_START.md` - Daily operations
   - `103_PERCEPTION_STATUS_TROUBLESHOOTING.md` - Debugging

3. **Tune parameters if needed:**
   - Edit `audio_params.yaml` for timing adjustments
   - Edit `gesture_params.yaml` for gesture behavior

---

**Document Version:** 1.0  
**Last Updated:** December 21, 2025  
**Status:** Complete installation guide  
**Estimated Total Time:** 2-3 hours

