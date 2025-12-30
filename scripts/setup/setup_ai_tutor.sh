#!/bin/bash
# =============================================================================
# R2D2 AI Tutor System - Complete Setup Script
# =============================================================================
# Run with: sudo bash ~/dev/r2d2/scripts/setup/setup_ai_tutor.sh
#
# This script:
# 1. Installs the narrator service to systemd
# 2. Sets up passwordless control for voice activation
# 3. Verifies espeak is installed
# 4. Creates required data files
# =============================================================================

set -e  # Exit on error

echo "============================================="
echo "R2D2 AI Tutor System - Setup"
echo "============================================="
echo ""

# 1. Install narrator service
echo "[1/4] Installing narrator service..."
cp /home/severin/dev/r2d2/scripts/systemd/r2d2-narrator.service /etc/systemd/system/
systemctl daemon-reload
echo "✓ Narrator service installed"

# 2. Setup sudoers for passwordless control
echo ""
echo "[2/4] Setting up passwordless narrator control..."
SUDOERS_FILE="/etc/sudoers.d/r2d2-narrator"
cat > "$SUDOERS_FILE" << 'EOF'
# Allow severin to start/stop the r2d2-narrator service without password
# This enables voice-controlled learning mode activation
severin ALL=(ALL) NOPASSWD: /bin/systemctl start r2d2-narrator.service
severin ALL=(ALL) NOPASSWD: /bin/systemctl stop r2d2-narrator.service
EOF
chmod 0440 "$SUDOERS_FILE"

if visudo -c -f "$SUDOERS_FILE" > /dev/null 2>&1; then
    echo "✓ Sudoers entry created"
else
    echo "✗ Error: Invalid sudoers syntax. Removing file."
    rm -f "$SUDOERS_FILE"
    exit 1
fi

# 3. Check espeak
echo ""
echo "[3/4] Checking espeak installation..."
if command -v espeak &> /dev/null; then
    echo "✓ espeak is installed"
else
    echo "Installing espeak..."
    apt-get update && apt-get install -y espeak
    echo "✓ espeak installed"
fi

# 4. Create data files if missing
echo ""
echo "[4/4] Creating data files..."
DATA_DIR="/home/severin/dev/r2d2/data"
mkdir -p "$DATA_DIR"

# narrator_mode.txt
if [ ! -f "$DATA_DIR/narrator_mode.txt" ]; then
    echo "narrator" > "$DATA_DIR/narrator_mode.txt"
    chown severin:severin "$DATA_DIR/narrator_mode.txt"
    echo "✓ Created narrator_mode.txt"
else
    echo "✓ narrator_mode.txt exists"
fi

# tutor_mode_active.txt
if [ ! -f "$DATA_DIR/tutor_mode_active.txt" ]; then
    echo "false" > "$DATA_DIR/tutor_mode_active.txt"
    chown severin:severin "$DATA_DIR/tutor_mode_active.txt"
    echo "✓ Created tutor_mode_active.txt"
else
    echo "✓ tutor_mode_active.txt exists"
fi

# coding_context.json
if [ ! -f "$DATA_DIR/coding_context.json" ]; then
    echo "{}" > "$DATA_DIR/coding_context.json"
    chown severin:severin "$DATA_DIR/coding_context.json"
    echo "✓ Created coding_context.json"
else
    echo "✓ coding_context.json exists"
fi

# coding_live.md
if [ ! -f "$DATA_DIR/coding_live.md" ]; then
    cat > "$DATA_DIR/coding_live.md" << 'MDEOF'
## Current Action
AI Tutor system initialized

## What This Means
The R2D2 AI Tutor system is now ready. You can activate Learning Mode 
by saying "Turn on learning mode" or Tutor Mode by saying "Be my tutor".

## BI Analogy
Like initializing a new ETL pipeline - the components are ready to process data.

## Category
System > Initialization | Understanding: 5
MDEOF
    chown severin:severin "$DATA_DIR/coding_live.md"
    echo "✓ Created coding_live.md"
else
    echo "✓ coding_live.md exists"
fi

echo ""
echo "============================================="
echo "✅ AI Tutor System Setup Complete!"
echo "============================================="
echo ""
echo "Voice commands now available:"
echo "  • 'Turn on learning mode' - Start coding narrator"
echo "  • 'Learning off' - Stop coding narrator"
echo "  • 'Be my tutor' - Activate teaching mode"
echo "  • 'Tutor off' - Deactivate teaching mode"
echo ""
echo "Test the narrator service:"
echo "  sudo systemctl start r2d2-narrator.service"
echo "  sudo systemctl status r2d2-narrator.service"
echo "  sudo systemctl stop r2d2-narrator.service"
echo ""



