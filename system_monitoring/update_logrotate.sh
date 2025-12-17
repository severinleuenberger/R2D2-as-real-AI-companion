#!/bin/bash
#
# Update logrotate configuration to 3-day retention
#

echo "Updating freeze monitor log rotation to 3 days..."
sudo cp /home/severin/dev/r2d2/system_monitoring/freeze-monitor.logrotate /etc/logrotate.d/freeze-monitor

echo "Verifying configuration..."
grep -A 1 "Keep" /etc/logrotate.d/freeze-monitor

echo ""
echo "✓ Log rotation updated to 3 days"
echo "Older logs will be automatically deleted after 3 days"

