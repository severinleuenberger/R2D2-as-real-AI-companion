#!/bin/bash
# Extended passwordless sudo for R2D2 service installation and control
# This extends the existing passwordless sudo to include service installation

echo "Setting up extended passwordless sudo for R2D2 services..."
echo ""

# Create extended sudoers file that includes installation commands
SUDOERS_RULE="severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-*, /bin/systemctl stop r2d2-*, /bin/systemctl restart r2d2-*, /bin/cp /home/severin/dev/r2d2/r2d2-*.service /etc/systemd/system/, /bin/chmod 644 /etc/systemd/system/r2d2-*.service, /bin/systemctl daemon-reload, /bin/systemctl enable r2d2-*.service"

echo "$SUDOERS_RULE" | sudo tee /etc/sudoers.d/r2d2-services-extended > /dev/null

# Set correct permissions
sudo chmod 0440 /etc/sudoers.d/r2d2-services-extended

# Verify syntax
if sudo visudo -c -f /etc/sudoers.d/r2d2-services-extended; then
    echo ""
    echo "✅ Extended passwordless sudo configured successfully!"
    echo ""
    echo "You can now install and manage R2D2 services without a password."
else
    echo ""
    echo "❌ Error: sudoers file syntax is invalid!"
    echo "Please check /etc/sudoers.d/r2d2-services-extended manually"
    exit 1
fi



