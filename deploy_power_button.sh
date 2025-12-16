#!/bin/bash
# Deploy updated power button script to system location

SOURCE_FILE="/home/severin/dev/r2d2/r2d2_power_button.py"
TARGET_FILE="/usr/local/bin/r2d2_power_button.py"

echo "Deploying power button script..."
echo "Source: $SOURCE_FILE"
echo "Target: $TARGET_FILE"

# Copy file and set permissions
sudo cp "$SOURCE_FILE" "$TARGET_FILE"
sudo chmod +x "$TARGET_FILE"
sudo chown root:root "$TARGET_FILE"

echo "✓ File deployed successfully"
echo ""
echo "Restarting service..."
sudo systemctl restart r2d2-powerbutton.service

echo "✓ Service restarted"
echo ""
echo "Checking service status..."
sudo systemctl status r2d2-powerbutton.service --no-pager -l

