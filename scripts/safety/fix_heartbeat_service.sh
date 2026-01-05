#!/bin/bash
# Fix heartbeat service ExecStart path
# Run with: sudo ./fix_heartbeat_service.sh

set -e

echo "Fixing r2d2-heartbeat.service ExecStart path..."

# Update the service file
sed -i 's|ExecStart=/home/severin/dev/r2d2/start_heartbeat.sh|ExecStart=/home/severin/dev/r2d2/scripts/start/start_heartbeat.sh|' \
    /etc/systemd/system/r2d2-heartbeat.service

# Reload systemd
systemctl daemon-reload

# Restart service
systemctl restart r2d2-heartbeat.service

echo "✅ Heartbeat service fixed!"
echo ""
systemctl status r2d2-heartbeat.service --no-pager

