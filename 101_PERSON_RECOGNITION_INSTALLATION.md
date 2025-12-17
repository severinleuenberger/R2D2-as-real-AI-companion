# R2D2 Person Recognition System - Installation Guide
## Step-by-Step Setup for Complete Person Recognition System

**Date:** December 17, 2025  
**Status:** Complete Installation Guide  
**Platform:** NVIDIA Jetson AGX Orin 64GB + ROS 2 Humble  
**Estimated Time:** 120-180 minutes

---

## Prerequisites

Before starting installation, verify you have:

### Hardware Requirements

✅ **NVIDIA Jetson AGX Orin 64GB** (or compatible Jetson)
- JetPack 6.x installed
- At least 10GB free storage
- Dual USB 3.0 ports available

✅ **OAK-D Lite Camera** (Luxonis)
- USB 3.0 USB-C to USB-A cable included
- Auto-focus version recommended
- No special drivers needed (DepthAI SDK)

✅ **Speaker + PAM8403 Amplifier**
- 8Ω 3W speaker (or similar)
- PAM8403 Class-D amplifier board
- Jumper wires for connections

✅ **RGB LED (Optional)**
- Common cathode or common anode RGB LED
- 3× 220Ω current-limiting resistors
- Jumper wires for GPIO connections

### Software Requirements

✅ **ROS 2 Humble** installed and working
```bash
ros2 --version
# Should show: ROS 2 Humble
```

✅ **Python 3.10** (should be pre-installed on Jetson)
```bash
python3 --version
# Should show: Python 3.10.x
```

✅ **Git** (for cloning/updates)
```bash
git --version
```

### Cross-References

- **Reference Documentation:** See [`100_PERSON_RECOGNITION_REFERENCE.md`](100_PERSON_RECOGNITION_REFERENCE.md) for architecture details
- **Quick Start:** See [`102_PERSON_RECOGNITION_QUICK_START.md`](102_PERSON_RECOGNITION_QUICK_START.md) for daily use

---

## Part 1: Hardware Setup (30 minutes)

### Step 1.1: Connect OAK-D Lite Camera

**Physical Connection:**
```bash
# Plug OAK-D Lite into USB 3.0 port on Jetson
# Use direct USB port (not through hub)
# Blue USB connector = USB 3.0
```

**Verify USB Detection:**
```bash
# Check USB devices
lsusb | grep Movidius

# Expected output:
# Bus 001 Device 003: ID 03e7:2485 Intel Movidius MyriadX
```

If not detected:
- Try different USB port
- Check cable quality
- Check `dmesg | tail -20` for USB errors

### Step 1.2: Wire PAM8403 Amplifier

**Wiring Diagram:**
```
Jetson 40-pin header (J30):
  Pin 2  (5V)   → PAM8403 +5V (power supply)
  Pin 6  (GND)  → PAM8403 GND (power ground)

Jetson audio panel header (J511):
  Pin 9  (HPO_L, left analog audio out) → PAM8403 LIN (left audio input)
  Pin 2  (AGND, audio ground)          → PAM8403 GND (audio signal ground)

PAM8403 speaker output (class-D amplifier):
  L+ → speaker + (red wire)
  L− → speaker − (black wire)
```

**Important Notes:**
- PAM8403 requires 5V power from Jetson Pin 2
- Audio signal comes from J511 (small header near audio jack)
- Double-check polarity: +5V and GND must not be reversed
- Use short wires (<6 inches) to minimize interference

### Step 1.3: Wire RGB LED (Optional)

**GPIO Connections:**
```
Jetson 40-pin header (J30):
  Pin 11 (GPIO 17) → 220Ω resistor → RED LED anode
  Pin 13 (GPIO 27) → 220Ω resistor → GREEN LED anode
  Pin 15 (GPIO 22) → 220Ω resistor → BLUE LED anode
  Pin 14 (GND)     → Common cathode (all LED cathodes connected)
```

**LED Configuration:**
- Common cathode RGB LED (most common)
- If using common anode, invert GPIO logic in software
- 220Ω resistors prevent LED burnout (3.3V GPIO)

### Step 1.4: Verify Hardware

**Camera Test:**
```bash
# Install basic USB tools if needed
sudo apt install usbutils

# Verify camera detection
lsusb | grep -i movidius
# Should show: 03e7:2485
```

**Audio Test (Simple):**
```bash
# List audio devices
aplay -l

# Should show Card 1 (APE/I2S)
# Example output:
# card 1: tegra [NVIDIA Jetson AGX Orin APE], device 0: ...
```

---

## Part 2: Software Setup (40 minutes)

### Step 2.1: Install System Dependencies

**System Libraries:**
```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-dev \
  python3-pip \
  python3-venv \
  libjpeg-dev \
  libpng-dev \
  libopenexr-dev \
  libtbb-dev \
  libatlas-base-dev \
  libgl1-mesa-glx \
  portaudio19-dev \
  libasound2-dev \
  alsa-utils \
  pulseaudio-utils \
  ffmpeg

# This takes ~5-10 minutes
```

### Step 2.2: Create DepthAI Virtual Environment

**Create and Activate:**
```bash
# Create dedicated virtualenv for DepthAI/OAK-D
cd ~
python3 -m venv depthai_env

# Activate it
source ~/depthai_env/bin/activate

# You should see: (depthai_env) in your prompt
```

**Important:** This virtualenv must be activated before using the camera.

### Step 2.3: Install DepthAI SDK

**Official Luxonis Installation:**
```bash
# Install Luxonis dependencies (official script)
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash

# Activate virtualenv
source ~/depthai_env/bin/activate

# Clone DepthAI Python
cd ~
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python

# Install requirements (includes DepthAI 2.31.0.0)
python3 examples/install_requirements.py

# This takes ~10-15 minutes on Jetson
```

### Step 2.4: Set Critical Environment Variable

**ARM Configuration:**
```bash
# Add to ~/.bashrc (CRITICAL for ARM/Jetson)
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
source ~/.bashrc

# Verify
echo $OPENBLAS_CORETYPE
# Should show: ARMV8
```

**Why This Matters:**
- Prevents "illegal instruction" errors on ARM processors
- Required for numpy/scipy operations
- Must be set before running any camera code

### Step 2.5: Verify DepthAI Installation

**Test Camera Access:**
```bash
source ~/depthai_env/bin/activate
python3 << 'EOF'
import depthai as dai
print(f"DepthAI version: {dai.__version__}")
devices = dai.XLinkConnection.getAllConnectedDevices()
print(f"Connected devices: {len(devices)}")
if len(devices) > 0:
    print("✓ OAK-D camera detected!")
else:
    print("✗ No camera detected")
EOF

# Expected output:
# DepthAI version: 2.31.0.0
# Connected devices: 1
# ✓ OAK-D camera detected!
```

### Step 2.6: Configure ALSA Audio

**Create ALSA Configuration File:**
```bash
sudo tee /etc/asound.conf > /dev/null << 'EOF'
# R2D2 Audio Configuration: PAM8403 Speaker via I2S (Card 1)

# Default PCM device: route to APE I2S
pcm.!default {
    type asym
    playback.pcm "speaker_out"
    capture.pcm "speaker_in"
}

# Speaker output via APE I2S (Card 1, Device 0)
pcm.speaker_out {
    type dmix
    ipc_key 1234
    slave {
        pcm "hw:1,0"           # Card 1 (APE), Device 0 (I2S)
        rate 44100             # Sample rate
        channels 2             # Stereo (will use left only)
        period_size 4096
        buffer_size 65536
    }
    bindings {
        0 0                    # Map dmix channel 0 to left speaker
        1 1                    # Map dmix channel 1 to right
    }
}

# Capture placeholder
pcm.speaker_in {
    type hw
    card 1
    device 0
}

# Control mixer
ctl.!default {
    type hw
    card 1
}
EOF
```

**Test Audio Playback:**
```bash
# Test speaker with tone
speaker-test -t wav -c 2 -D hw:1,0

# Press Ctrl+C to stop
# You should hear left/right test sounds
```

If no sound:
- Check PAM8403 wiring (5V power, audio input, speaker connections)
- Verify J511 header connections
- Check `aplay -l` shows Card 1

---

## Part 3: ROS 2 Workspace Setup (20 minutes)

### Step 3.1: Build ROS 2 Packages

**Navigate to Workspace:**
```bash
cd ~/dev/r2d2/ros2_ws
```

**Source ROS 2:**
```bash
source /opt/ros/humble/setup.bash
```

**Build Perception Package:**
```bash
colcon build --packages-select r2d2_perception --symlink-install

# Expected output:
# Starting >>> r2d2_perception
# Finished <<< r2d2_perception [1.79s]
# Summary: 1 package finished [2.33s]
```

**Build Audio Package:**
```bash
colcon build --packages-select r2d2_audio --symlink-install

# Expected output:
# Starting >>> r2d2_audio
# Finished <<< r2d2_audio [1.52s]
```

**Build Camera Package:**
```bash
colcon build --packages-select r2d2_camera --symlink-install

# Expected output:
# Starting >>> r2d2_camera
# Finished <<< r2d2_camera [1.23s]
```

**Build Bringup Package:**
```bash
colcon build --packages-select r2d2_bringup --symlink-install
```

**Source Installation:**
```bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Verify packages
ros2 pkg list | grep r2d2
# Should show:
#   r2d2_audio
#   r2d2_bringup
#   r2d2_camera
#   r2d2_perception
```

---

## Part 4: Perception Pipeline Setup (15 minutes)

### Step 4.1: Launch Camera + Perception

**Integrated Launch:**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

**What's Running:**
- `camera_node`: Publishes `/oak/rgb/image_raw` (30 Hz)
- `image_listener`: Processes frames, publishes:
  - `/r2d2/perception/brightness` (Float32, ~13 Hz)
  - `/r2d2/perception/face_count` (Int32, ~13 Hz)

**Verify It's Working (in another terminal):**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

# Check face detection
ros2 topic echo /r2d2/perception/face_count
# Should show: data: 0, 1, 2, etc. (number of faces detected)

# Check brightness
ros2 topic echo /r2d2/perception/brightness
# Should show: data: 112.5, 128.3, etc.
```

**Expected Performance:**
- Frame processing rate: 12-13 FPS
- CPU usage: 8-10% (one core)
- Memory usage: ~200 MB

---

## Part 5: Face Recognition Training (30 minutes)

### Step 5.1: Prepare Training Directory

**Create Data Directory:**
```bash
mkdir -p ~/dev/r2d2/data/face_recognition/models
chmod 755 ~/dev/r2d2/data/face_recognition
```

### Step 5.2: Capture Training Data

**Navigate to Training Scripts:**
```bash
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
```

**Run Capture Script:**
```bash
python3 1_capture_training_data.py
```

**Menu Selection:**
```
1. Train new person
```

**Follow On-Screen Instructions:**
- Task 1: Bright light, 1 meter (20 seconds)
- Task 2: Bright light, 2 meters (20 seconds)
- Task 3: Low light, 3 meters (20 seconds)
- Task 4: Low light, 5 meters (20 seconds)

**Tips for Better Recognition:**
- Vary lighting (bright, low light, natural, artificial)
- Capture from different angles (straight, 45°, 90°)
- Include different expressions (neutral, smile, looking down)
- Some images with partial occlusions (turned head, glasses)

**Expected Result:**
```
✓ Captured ~80 images
✓ Saved to ~/dev/r2d2/data/face_recognition/severin/
```

### Step 5.3: Train the Model

**Run Training Script:**
```bash
python3 2_train_recognizer.py
```

**What It Does:**
- Reads all images from capture directory
- Extracts faces using Haar Cascade
- Trains LBPH recognizer
- Saves model to `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- Duration: 30-60 seconds

**Expected Output:**
```
Training LBPH recognizer...
Processing 80 images...
Training complete!
Model saved to: ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
```

**Verify Model:**
```bash
ls -lh ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
# Should show: ~33 MB file
```

### Step 5.4: Test the Model

**Run Test Script:**
```bash
python3 3_test_recognizer_demo.py
```

**What It Shows:**
- Recognition accuracy statistics
- Confidence scores for test images
- Suggestions for improving accuracy

---

## Part 6: ROS 2 Face Recognition Integration (10 minutes)

### Step 6.1: Launch with Face Recognition

**Stop Previous Launch** (Ctrl+C in Terminal 1)

**Launch with Recognition Enabled:**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Requirements:**
- Trained model must exist at `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`
- If model not found, node logs warning and continues without recognition

### Step 6.2: Verify Recognition

**Monitor Person ID (in another terminal):**
```bash
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

ros2 topic echo /r2d2/perception/person_id
# Expected output:
# data: target_person
# ---
# data: unknown
# ---
# data: target_person
```

**Check Publishing Rate:**
```bash
ros2 topic hz /r2d2/perception/person_id
# Expected (with frame_skip=2):
# average rate: 6.50
```

**Expected Performance:**
- Recognition rate: 6.5 Hz (with frame_skip=2)
- CPU usage: 10-15%
- Latency: <100 ms

---

## Part 7: Audio Notification System (15 minutes)

### Step 7.1: Build Audio Package

**If not already built:**
```bash
cd ~/dev/r2d2/ros2_ws
colcon build --packages-select r2d2_audio
source install/setup.bash
```

### Step 7.2: Launch Audio Notifications (Manual)

**Option A: Manual Launch for Testing:**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**What You'll Hear:**
- 🔊 **"Hello!"** MP3 when face enters frame (unknown → recognized)
- ⏸ **Silent** while continuously recognized
- 🔔 **"Oh, I lost you!"** MP3 after >20 seconds absence
- 🔊 **"Hello!"** MP3 when you return (lost → recognized)

### Step 7.3: Configure Volume

**Default Volume:** 0.05 (5% - very quiet)

**Adjust Volume (Runtime):**
```bash
# In another terminal
source /opt/ros/humble/setup.bash
source ~/dev/r2d2/ros2_ws/install/setup.bash

ros2 param set /audio_notification_node audio_volume 0.3
```

**Volume Levels:**
- `0.05` = 5% (current default, very quiet)
- `0.1` = 10% (quiet, library/office)
- `0.2` = 20% (moderate, home use)
- `0.5` = 50% (loud, normal room)
- `1.0` = 100% (maximum)

### Step 7.4: Test Audio System

**Test Recognition Alert:**
```bash
# In Terminal 1: Monitor status
ros2 topic echo /r2d2/audio/person_status --no-arr

# In Terminal 2: Simulate recognition
ros2 topic pub --once /r2d2/perception/person_id std_msgs/String "{data: target_person}"
```

**Expected:**
- 🔊 "Hello!" plays
- Status changes to: `{"status": "red", "person_identity": "target_person", ...}`

---

## Part 8: Production Deployment (20 minutes)

### Step 8.1: Create Systemd Service

**Copy Service File:**
```bash
sudo cp /home/severin/dev/r2d2/r2d2-audio-notification.service /etc/systemd/system/
```

**If service file doesn't exist, create it:**
```bash
sudo tee /etc/systemd/system/r2d2-audio-notification.service > /dev/null << 'EOF'
[Unit]
Description=R2D2 Audio Notification Service
After=network.target sound.target

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2
ExecStart=/home/severin/dev/r2d2/start_audio_service.sh
Restart=on-failure
RestartSec=5
StartLimitInterval=200
StartLimitBurst=3

[Install]
WantedBy=multi-user.target
EOF
```

**Create Startup Script:**
```bash
cat > ~/dev/r2d2/start_audio_service.sh << 'EOF'
#!/bin/bash
# R2D2 Audio Notification Service Startup Script

# Source ROS 2
source /opt/ros/humble/setup.bash
source /home/severin/dev/r2d2/ros2_ws/install/setup.bash

# Launch audio notification
exec ros2 launch r2d2_audio audio_notification.launch.py
EOF

chmod +x ~/dev/r2d2/start_audio_service.sh
```

### Step 8.2: Enable and Start Service

**Enable Service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable r2d2-audio-notification.service
```

**Start Service:**
```bash
sudo systemctl start r2d2-audio-notification.service
```

**Check Status:**
```bash
sudo systemctl status r2d2-audio-notification.service
# Should show: active (running)
```

**View Logs:**
```bash
sudo journalctl -u r2d2-audio-notification.service -f
```

### Step 8.3: Complete System Startup

**Full R2D2 Startup Sequence:**

**Terminal 1: Camera + Perception + Recognition**
```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash

ros2 launch r2d2_bringup r2d2_camera_perception.launch.py \
  enable_face_recognition:=true
```

**Terminal 2: Audio Notifications (if not using service)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_audio audio_notification.launch.py
```

**Terminal 3: LED Node (optional)**
```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 run r2d2_audio status_led_node
```

**Terminal 4: Monitoring**
```bash
# Watch person recognition
ros2 topic echo /r2d2/perception/person_id

# Watch status messages
ros2 topic echo /r2d2/audio/person_status --no-arr

# Check topic rates
ros2 topic hz /r2d2/perception/person_id
ros2 topic hz /r2d2/audio/person_status
```

---

## Part 9: Post-Installation Verification (10 minutes)

### Step 9.1: Verification Checklist

**Hardware:**
- [ ] OAK-D Lite detected via `lsusb`
- [ ] OAK-D shows up in DepthAI device list
- [ ] PAM8403 speaker produces sound
- [ ] RGB LED lights up (if installed)

**Software:**
- [ ] DepthAI virtualenv created and activates
- [ ] All ROS 2 packages built without errors
- [ ] ALSA configuration created (`/etc/asound.conf`)
- [ ] OPENBLAS_CORETYPE=ARMV8 set in ~/.bashrc

**Training:**
- [ ] Training images captured (~80 images)
- [ ] LBPH model trained and saved (~33 MB XML file)
- [ ] Model location: `~/dev/r2d2/data/face_recognition/models/severin_lbph.xml`

**Functionality:**
- [ ] Camera publishes frames to `/oak/rgb/image_raw`
- [ ] Perception publishes face count and brightness
- [ ] Recognition publishes person ID when enabled
- [ ] Audio notification plays "Hello!" on recognition
- [ ] Audio notification plays "Oh, I lost you!" on loss
- [ ] Status messages published to `/r2d2/audio/person_status`
- [ ] LED changes color based on status (if installed)

**ROS 2 Integration:**
- [ ] Nodes show in `ros2 node list`
- [ ] Topics show in `ros2 topic list`
- [ ] Topic rates meet expectations (30 Hz camera, 13 Hz perception, 6.5 Hz recognition)
- [ ] Service auto-starts on boot (if configured)

### Step 9.2: Post-Reboot Verification

**After Rebooting Jetson:**
```bash
# 1. Wait 30 seconds for system to stabilize
sleep 30

# 2. Check service status (if configured)
sudo systemctl status r2d2-audio-notification.service
# Should show: active (running)

# 3. Launch camera + perception
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true

# 4. In another terminal, verify topics
ros2 topic list | grep r2d2
# Should show all perception and audio topics

# 5. Test recognition
# Stand in front of camera - should hear "Hello!" beep
```

---

## Troubleshooting Installation Issues

### Issue: DepthAI Installation Fails

**Symptom:** pip install fails or "illegal instruction" errors

**Solutions:**
1. Ensure OPENBLAS_CORETYPE=ARMV8 is set:
   ```bash
   echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
   source ~/.bashrc
   ```
2. Use official Luxonis installation script
3. Install in virtual environment (not system Python)
4. Check Python version is 3.10

### Issue: ROS 2 Package Build Fails

**Symptom:** `colcon build` fails with errors

**Solutions:**
```bash
# Clean build
cd ~/dev/r2d2/ros2_ws
rm -rf build/ install/ log/

# Rebuild
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Issue: Camera Not Detected

**Symptom:** `lsusb` doesn't show OAK-D

**Solutions:**
1. Try different USB port (use USB 3.0 directly, not through hub)
2. Check cable quality
3. Verify with: `python3 -c "import depthai as dai; print(len(dai.XLinkConnection.getAllConnectedDevices()))"`
4. Check `dmesg | tail -20` for USB errors

### Issue: No Audio Output

**Symptom:** Speaker test fails or no sound

**Solutions:**
1. Check PAM8403 wiring (5V power, audio input, GND)
2. Verify J511 header connections (Pin 9: HPO_L, Pin 2: AGND)
3. Test with: `speaker-test -t wav -c 2 -D hw:1,0`
4. Check ALSA config: `cat /etc/asound.conf`

### Issue: Training Fails

**Symptom:** Cannot capture images or train model

**Solutions:**
1. Ensure camera is detected: `lsusb | grep Movidius`
2. Activate virtualenv: `source ~/depthai_env/bin/activate`
3. Set environment: `export OPENBLAS_CORETYPE=ARMV8`
4. Check directory permissions: `ls -la ~/dev/r2d2/data/face_recognition/`

---

## Next Steps

After successful installation:

1. **Read Reference Documentation:** [`100_PERSON_RECOGNITION_REFERENCE.md`](100_PERSON_RECOGNITION_REFERENCE.md)
2. **Read Quick Start Guide:** [`102_PERSON_RECOGNITION_QUICK_START.md`](102_PERSON_RECOGNITION_QUICK_START.md)
3. **Experiment with recognition:** Try different lighting and distances
4. **Tune parameters:** Adjust confidence threshold and timing
5. **Integrate with other systems:** Connect to navigation, speech, etc.

---

**Installation Guide Version:** 1.0  
**Last Updated:** December 17, 2025  
**Estimated Installation Time:** 120-180 minutes  
**Status:** Complete and tested  
**Hardware:** OAK-D Lite + PAM8403 Speaker + Optional RGB LED

