#!/bin/bash

# R2D2 Face Recognition - Quick Command Reference
# Copy-paste ready commands for all common tasks

# ============================================================================
# ENVIRONMENT SETUP (Do this first!)
# ============================================================================

# Set working directory and activate environment
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8

# ============================================================================
# TRAINING & TESTING (Menu-driven)
# ============================================================================

# Launch training/testing menu (7 options)
python3 train_manager.py

# Menu options:
#   [1] Train new person (with 4-task interactive training)
#   [2] Add more pictures to existing person
#   [3] Retrain model from existing images
#   [4] Test accuracy at different distances
#   [5] Real-time 30-second test (INSTANT FEEDBACK!)
#   [6] List all people and models
#   [7] Delete person (safe deletion with confirmation)
#   [0] Exit

# ============================================================================
# BACKGROUND SERVICE
# ============================================================================

# Start service for "severin"
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition

# Stop service
python3 face_recognition_service.py stop

# Check current status
python3 face_recognition_service.py status

# View last 50 lines of log
python3 face_recognition_service.py logs 50

# View last 100 lines of log
python3 face_recognition_service.py logs 100

# ============================================================================
# SYSTEMD SERVICE (Auto-start on boot)
# ============================================================================

# Install systemd service
sudo cp r2d2-face-recognition.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable r2d2-face-recognition

# Start service now
sudo systemctl start r2d2-face-recognition

# Check systemd service status
sudo systemctl status r2d2-face-recognition

# View systemd logs (follow mode)
sudo journalctl -u r2d2-face-recognition -f

# View last 100 systemd logs
sudo journalctl -u r2d2-face-recognition -n 100

# Restart systemd service
sudo systemctl restart r2d2-face-recognition

# Stop systemd service
sudo systemctl stop r2d2-face-recognition

# Disable auto-start
sudo systemctl disable r2d2-face-recognition

# ============================================================================
# TESTING & VERIFICATION
# ============================================================================

# Run complete system test suite (7 tests)
python3 test_complete_system.py

# Test LED controller specifically
python3 -c "from led_controller import create_led_controller; led = create_led_controller('text'); led.set_recognized('severin'); print(); led.set_unrecognized(); print(); led.set_error(); print()"

# Verify camera is working
python3 -c "import depthai as dai; print('✓ Camera OK')"

# Check model file
ls -lh ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml

# Count training images
find ~/dev/r2d2/data/face_recognition/severin -name '*.jpg' | wc -l

# ============================================================================
# MONITORING & DEBUGGING
# ============================================================================

# Monitor CPU usage while service runs
top -p $(pgrep -f face_recognition_service.py)

# More detailed CPU monitoring
htop -p $(pgrep -f face_recognition_service.py)

# Watch status file updates
watch -n 1 'cat ~/.r2d2_face_recognition_status.json | python3 -m json.tool'

# Tail log file (follow mode)
tail -f ~/.r2d2_face_recognition.log

# Search logs for errors
grep "ERROR" ~/.r2d2_face_recognition.log

# Search logs for recognition events
grep "RECOGNIZED" ~/.r2d2_face_recognition.log

# ============================================================================
# DATA MANAGEMENT
# ============================================================================

# List all trained people
ls ~/dev/r2d2/data/face_recognition/

# List all trained models
ls -lh ~/dev/r2d2/data/face_recognition/models/

# Count images for a person
ls ~/dev/r2d2/data/face_recognition/severin/ | wc -l

# View recent training images
ls -lt ~/dev/r2d2/data/face_recognition/severin/*.jpg | head -20

# Check total training data size
du -sh ~/dev/r2d2/data/face_recognition/

# ============================================================================
# GIT OPERATIONS
# ============================================================================

# Check what's tracked by git
git -C ~/dev/r2d2 status

# Verify training images are protected (not staged)
git -C ~/dev/r2d2 status | grep "face_recognition"

# Verify models are being tracked
git -C ~/dev/r2d2 ls-files | grep "face_recognition/models"

# Show what would be committed
git -C ~/dev/r2d2 diff --cached --name-only | grep "face_recognition"

# ============================================================================
# TROUBLESHOOTING
# ============================================================================

# If service won't start - check logs first
python3 face_recognition_service.py logs 100
tail -f ~/.r2d2_face_recognition.log

# If camera not found - verify DepthAI
python3 -c "import depthai as dai; devices = dai.Device.getAllAvailableDevices(); print(f'Found {len(devices)} device(s)')"

# If low recognition - test with real-time feedback
python3 train_manager.py  # Then select [5]

# If high CPU - reduce frame skip rate
# Edit face_recognition_service.py and change cpu_limit=0.10 (from 0.15)

# If model missing - check and retrain
ls ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml
python3 train_manager.py  # Then select [3]

# ============================================================================
# USEFUL ALIASES (Add to ~/.bashrc for convenience)
# ============================================================================

# Add these to ~/.bashrc if you want quick commands:
# alias r2d2_train='cd ~/dev/r2d2/tests/face_recognition && source ~/depthai_env/bin/activate && python3 train_manager.py'
# alias r2d2_service='cd ~/dev/r2d2/tests/face_recognition && source ~/depthai_env/bin/activate && python3 face_recognition_service.py'
# alias r2d2_test='cd ~/dev/r2d2/tests/face_recognition && source ~/depthai_env/bin/activate && python3 test_complete_system.py'
# alias r2d2_logs='python3 ~/dev/r2d2/tests/face_recognition/face_recognition_service.py logs 50'
# alias r2d2_status='python3 ~/dev/r2d2/tests/face_recognition/face_recognition_service.py status'

# Then use simply:
# r2d2_train       # Start training menu
# r2d2_service start severin
# r2d2_status
# r2d2_logs

# ============================================================================
# COMMON WORKFLOWS
# ============================================================================

# WORKFLOW 1: Train a new person
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 train_manager.py
# Select [1], enter name, complete 4 tasks

# WORKFLOW 2: Quick test (30 seconds with instant feedback)
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 train_manager.py
# Select [5]

# WORKFLOW 3: Start service and monitor
# Terminal 1:
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 face_recognition_service.py start severin ~/dev/r2d2/data/face_recognition

# Terminal 2:
python3 ~/dev/r2d2/tests/face_recognition/face_recognition_service.py status

# Terminal 3:
tail -f ~/.r2d2_face_recognition.log

# WORKFLOW 4: Add more training images
cd ~/dev/r2d2/tests/face_recognition
source ~/depthai_env/bin/activate
export OPENBLAS_CORETYPE=ARMV8
python3 train_manager.py
# Select [2], choose person, capture images
# Then select [3] to retrain model

# ============================================================================
# FILES YOU NEED TO KNOW ABOUT
# ============================================================================

# Core files:
# ~/dev/r2d2/tests/face_recognition/face_recognition_service.py
# ~/dev/r2d2/tests/face_recognition/train_manager.py
# ~/dev/r2d2/tests/face_recognition/led_controller.py

# Configuration:
# ~/dev/r2d2/tests/face_recognition/r2d2-face-recognition.service

# Documentation:
# ~/dev/r2d2/tests/face_recognition/SYSTEM_DOCUMENTATION.md
# ~/dev/r2d2/tests/face_recognition/QUICK_START.md
# ~/dev/r2d2/tests/face_recognition/INTEGRATION_GUIDE.md

# Data:
# ~/dev/r2d2/data/face_recognition/severin/  (training images)
# ~/dev/r2d2/data/face_recognition/models/severin_lbph.xml  (trained model)

# Logs:
# ~/.r2d2_face_recognition.log  (service logs)
# ~/.r2d2_face_recognition_status.json  (status file)

# ============================================================================
# QUICK REFERENCE TABLE
# ============================================================================

# Command                                          What it does
# ─────────────────────────────────────────────────────────────────────────
python3 train_manager.py                          Open training menu
python3 face_recognition_service.py start         Start background service
python3 face_recognition_service.py stop          Stop background service
python3 face_recognition_service.py status        Check service status
python3 face_recognition_service.py logs 50       View last 50 log lines
python3 test_complete_system.py                   Run all system tests
sudo systemctl start r2d2-face-recognition        Start systemd service
sudo systemctl status r2d2-face-recognition       Check systemd status
sudo journalctl -u r2d2-face-recognition -f       Follow systemd logs
tail -f ~/.r2d2_face_recognition.log              Follow service logs
python3 face_recognition_service.py status | head Check last recognition
cat ~/.r2d2_face_recognition_status.json          Check status JSON file

# ============================================================================
# NOTES
# ============================================================================

# Always activate environment first:
# source ~/depthai_env/bin/activate
# export OPENBLAS_CORETYPE=ARMV8

# Service uses these settings:
# - Resolution: 1280×720 (CPU optimized)
# - FPS: 15 (CPU optimized)
# - CPU Limit: 10-15%
# - Confidence Threshold: 70 (optimal)
# - Recognition Timeout: 5 seconds

# Training data is protected by .gitignore
# Models are tracked by git for easy deployment

# To change LED type (text → GPIO → HTTP):
# Edit the line in face_recognition_service.py around line 80:
# self.led = led_module.create_led_controller('gpio', ...)
# or
# self.led = led_module.create_led_controller('http', ...)

# ============================================================================
