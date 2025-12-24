#!/bin/bash
# Setup passwordless sudo for R2D2 web dashboard service control
# This allows the web dashboard to start/stop/restart R2D2 services without a password

echo "Setting up passwordless sudo for R2D2 service control..."
echo ""

# Create sudoers file
echo "severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*" | sudo tee /etc/sudoers.d/r2d2-services

# Set correct permissions (sudoers files must be 0440)
sudo chmod 0440 /etc/sudoers.d/r2d2-services

# Verify syntax
echo ""
echo "Verifying sudoers file syntax..."
if sudo visudo -c -f /etc/sudoers.d/r2d2-services; then
    echo ""
    echo "✅ Passwordless sudo configured successfully!"
    echo ""
    echo "Testing passwordless sudo..."
    if sudo -n systemctl status r2d2-audio-notification.service > /dev/null 2>&1; then
        echo "✅ Test passed - passwordless sudo is working!"
        echo ""
        echo "The web dashboard can now control R2D2 services without a password."
    else
        echo "⚠️  Test failed - you may need to log out and back in for changes to take effect"
        echo "   Or run: newgrp - to refresh group permissions"
    fi
else
    echo ""
    echo "❌ Error: sudoers file syntax is invalid!"
    echo "Please check /etc/sudoers.d/r2d2-services manually"
    exit 1
fi

