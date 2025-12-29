#!/bin/bash
# R2D2 Disk Cleanup - runs daily via systemd timer
# Schedule: 30 minutes after boot, then every 24 hours
# Runtime: ~15-30 seconds
# Runs as: root (via systemd with HOME=/home/severin)
#
# What it cleans (all regeneratable):
# - pip cache (~50-200MB)
# - APT package cache (~100-400MB)
# - Journal logs older than 14 days (~20-50MB)
# - ROS logs older than 7 days (~20-50MB)
# - Cursor/VSCode debug logs (~0-900MB)

# Use explicit paths since script runs as root
USER_HOME="/home/severin"

echo "=== R2D2 Disk Cleanup Started: $(date) ==="

# Show disk usage before
BEFORE=$(df -h / | tail -1 | awk '{print $4}')
echo "Free space before: $BEFORE"

# Clean pip cache (regenerates on install)
echo "Cleaning pip cache..."
pip cache purge 2>/dev/null || true

# Clean APT package cache (regenerates on apt install)
echo "Cleaning APT cache..."
apt clean 2>/dev/null || true

# Rotate journal logs (keep 14 days)
echo "Vacuuming journal logs..."
journalctl --vacuum-time=14d 2>/dev/null || true

# Clean old ROS logs (keep 7 days)
echo "Cleaning old ROS logs..."
find "${USER_HOME}/.ros/log" -type f -name "*.log" -mtime +7 -delete 2>/dev/null || true

# Truncate Cursor debug log (can grow to 900MB+)
echo "Truncating IDE debug logs..."
truncate -s 0 "${USER_HOME}/.cursor/debug.log" 2>/dev/null || true

# Clean old VSCode server logs (older than 7 days)
find "${USER_HOME}/.vscode-server/data/logs" -type d -mtime +7 -exec rm -rf {} + 2>/dev/null || true

# Show disk usage after
AFTER=$(df -h / | tail -1 | awk '{print $4}')
echo ""
echo "=== Cleanup Complete ==="
echo "Free space before: $BEFORE"
echo "Free space after:  $AFTER"
echo "Timestamp: $(date)"

