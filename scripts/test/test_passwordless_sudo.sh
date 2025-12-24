#!/bin/bash
# Test script to verify passwordless sudo configuration

echo "Testing passwordless sudo configuration..."
echo ""

# Test if sudoers file exists
if [ -f /etc/sudoers.d/r2d2-services ]; then
    echo "✅ sudoers file exists: /etc/sudoers.d/r2d2-services"
    echo "   Permissions: $(ls -l /etc/sudoers.d/r2d2-services | awk '{print $1, $3, $4}')"
else
    echo "❌ sudoers file does not exist!"
    exit 1
fi

# Test passwordless sudo (this will prompt for password if not configured)
echo ""
echo "Testing passwordless sudo..."
echo "Running: sudo -n systemctl status r2d2-audio-notification.service"
if sudo -n systemctl status r2d2-audio-notification.service > /dev/null 2>&1; then
    echo "✅ Passwordless sudo WORKS!"
else
    echo "❌ Passwordless sudo FAILED"
    echo ""
    echo "This could mean:"
    echo "  1. The sudoers file syntax is incorrect"
    echo "  2. You need to log out and back in for changes to take effect"
    echo "  3. Try running: newgrp - (to refresh group permissions)"
    echo ""
    echo "To check the sudoers file content, run:"
    echo "  sudo cat /etc/sudoers.d/r2d2-services"
    echo ""
    echo "To check syntax, run:"
    echo "  sudo visudo -c -f /etc/sudoers.d/r2d2-services"
fi

