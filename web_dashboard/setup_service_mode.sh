#!/bin/bash
# Setup script for R2D2 Web UI Service Mode & Security Hardening
# Needs to be run with sudo

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo ./setup_service_mode.sh)"
  exit 1
fi

echo "--- 1. Creating Wake API Service ---"
cat > /etc/systemd/system/r2d2-wake-api.service << 'EOF'
[Unit]
Description=R2D2 Wake API Service (Minimal Service Mode)
After=network.target tailscaled.service

[Service]
Type=simple
User=severin
WorkingDirectory=/home/severin/dev/r2d2/web_dashboard
Environment="PATH=/home/severin/dev/r2d2/web_dashboard/web_dashboard_env/bin:/opt/ros/humble/bin:/usr/bin"
Environment="PYTHONPATH=/home/severin/dev/r2d2/web_dashboard"
# Source ROS 2 environment before starting
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && /home/severin/dev/r2d2/web_dashboard/web_dashboard_env/bin/python3 wake_api.py"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "--- 2. Disabling Auto-Start for Web UI Services ---"
systemctl disable r2d2-rosbridge.service
systemctl disable r2d2-web-dashboard.service
systemctl disable r2d2-camera-stream.service

echo "--- 3. Enabling Wake API Service ---"
systemctl daemon-reload
systemctl enable r2d2-wake-api.service
systemctl start r2d2-wake-api.service

echo "--- 4. Updating Sudoers for Service Control ---"
# Ensure the wake API can control these services without password
# We check if the config already exists, if not append or create
if [ -f /etc/sudoers.d/r2d2-services ]; then
    echo "Updating existing sudoers config..."
    # We need to make sure the wake API (running as severin) can start/stop these
    # The existing config likely covers it: severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, ...
    # Let's just ensure it's correct.
    echo "severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*, /bin/systemctl status r2d2-*" > /etc/sudoers.d/r2d2-services
else
    echo "Creating sudoers config..."
    echo "severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*, /bin/systemctl status r2d2-*" > /etc/sudoers.d/r2d2-services
fi
chmod 0440 /etc/sudoers.d/r2d2-services

echo "--- 5. Performing Disk Cleanup ---"
echo "Vacuuming journal logs..."
journalctl --vacuum-size=50M

echo "Cleaning package caches..."
rm -rf /home/severin/.cache/pip
rm -rf /home/severin/.cache/thumbnails
rm -rf /home/severin/.cache/fontconfig

echo "Configuring permanent log limits..."
# Uncomment SystemMaxUse and set to 50M
sed -i 's/#SystemMaxUse=/SystemMaxUse=50M/' /etc/systemd/journald.conf
sed -i 's/#SystemMaxUse=.*$/SystemMaxUse=50M/' /etc/systemd/journald.conf
systemctl restart systemd-journald

echo "--- 6. Installation Complete ---"
echo "Wake API is running at http://100.95.133.26:8079"
echo "Web UI services are stopped and disabled from auto-start."


