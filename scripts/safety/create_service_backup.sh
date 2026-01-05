#!/bin/bash
# Create timestamped backup of all R2D2 systemd service files
# Run with: sudo ./create_service_backup.sh

set -e

# Create timestamped backup directory
BACKUP_DIR=~/r2d2_service_backups/$(date +%Y%m%d_%H%M%S)_pre_webui_changes
mkdir -p "$BACKUP_DIR"

echo "Creating service backup..."
echo "Backup directory: $BACKUP_DIR"

# Backup all R2D2 service files
cp /etc/systemd/system/r2d2-*.service "$BACKUP_DIR/" || true

# Backup sudoers config
cp /etc/sudoers.d/r2d2-* "$BACKUP_DIR/" 2>/dev/null || true

# Document what was backed up
ls -lh "$BACKUP_DIR/" > "$BACKUP_DIR/MANIFEST.txt"
echo "" >> "$BACKUP_DIR/MANIFEST.txt"
echo "Backup created: $(date)" >> "$BACKUP_DIR/MANIFEST.txt"
echo "Current branch: $(cd /home/severin/dev/r2d2 && git branch --show-current)" >> "$BACKUP_DIR/MANIFEST.txt"
echo "Current commit: $(cd /home/severin/dev/r2d2 && git log -1 --oneline)" >> "$BACKUP_DIR/MANIFEST.txt"

# Make backup read-only for safety
chmod -R 444 "$BACKUP_DIR"/*

echo ""
echo "✅ Backup complete!"
echo ""
echo "Files backed up:"
ls -lh "$BACKUP_DIR/"
echo ""
echo "To restore:"
echo "  sudo cp $BACKUP_DIR/*.service /etc/systemd/system/"
echo "  sudo systemctl daemon-reload"

