#!/bin/bash
# Setup script to allow passwordless control of the r2d2-narrator service
# Run this once with: sudo bash setup_narrator_sudoers.sh

SUDOERS_FILE="/etc/sudoers.d/r2d2-narrator"

echo "Setting up sudoers entry for r2d2-narrator service..."

# Create the sudoers file
cat > "$SUDOERS_FILE" << 'EOF'
# Allow severin to start/stop the r2d2-narrator service without password
# This enables voice-controlled learning mode activation
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-narrator.service
severin ALL=(ALL) NOPASSWD: /bin/systemctl stop r2d2-narrator.service
EOF

# Set correct permissions (must be 0440 for sudoers files)
chmod 0440 "$SUDOERS_FILE"

# Verify syntax
if visudo -c -f "$SUDOERS_FILE" > /dev/null 2>&1; then
    echo "✓ Sudoers entry created successfully at $SUDOERS_FILE"
    echo "✓ You can now say 'Turn on learning mode' to R2D2!"
else
    echo "✗ Error: Invalid sudoers syntax. Removing file."
    rm -f "$SUDOERS_FILE"
    exit 1
fi

