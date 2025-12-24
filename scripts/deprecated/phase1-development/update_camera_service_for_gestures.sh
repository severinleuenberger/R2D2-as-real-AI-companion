#!/bin/bash
################################################################################
# Update Camera Perception Service - Enable Gesture Recognition
#
# This script updates the systemd service to enable gesture recognition
################################################################################

echo "Updating r2d2-camera-perception.service to enable gestures..."
echo ""

sudo tee /etc/systemd/system/r2d2-camera-perception.service > /dev/null <<'EOF'
[Unit]
Description=R2D2 Camera Perception Service (Face & Gesture Recognition)
After=network.target audio-notification.service
Documentation=file:///home/severin/dev/r2d2/QUICK_START.md

[Service]
Type=exec
User=severin
WorkingDirectory=/home/severin/dev/r2d2/ros2_ws
Environment="PATH=/home/severin/depthai_env/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="PYTHONUNBUFFERED=1"

# Source ROS 2 setup and launch camera with face and gesture recognition
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch r2d2_bringup r2d2_camera_perception.launch.py enable_face_recognition:=true target_person_name:=severin enable_gesture_recognition:=true'

# Restart policy
Restart=on-failure
RestartSec=5
StartLimitInterval=60s
StartLimitBurst=3

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=r2d2-camera

[Install]
WantedBy=multi-user.target
EOF

echo ""
echo "✓ Service file updated"
echo ""
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo ""
echo "Restarting service..."
sudo systemctl restart r2d2-camera-perception.service

sleep 3

echo ""
echo "Service status:"
systemctl status r2d2-camera-perception.service --no-pager | head -15

echo ""
echo "✅ Done! Gesture recognition is now enabled in the camera service."
echo ""
echo "Verify with:"
echo "  ros2 param get /image_listener enable_gesture_recognition"

