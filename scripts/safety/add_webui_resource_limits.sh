#!/bin/bash
# Add resource limits to WebUI systemd services for fault isolation
# Run with: sudo ./add_webui_resource_limits.sh

set -e

echo "Adding resource limits to WebUI services..."
echo ""

# Services to update
SERVICES=(
    "r2d2-web-dashboard.service"
    "r2d2-rosbridge.service"
)

for SERVICE in "${SERVICES[@]}"; do
    SERVICE_FILE="/etc/systemd/system/$SERVICE"
    
    if [ ! -f "$SERVICE_FILE" ]; then
        echo "⚠️  Skipping $SERVICE (file not found)"
        continue
    fi
    
    echo "Updating: $SERVICE"
    
    # Check if resource limits already exist
    if grep -q "CPUQuota" "$SERVICE_FILE"; then
        echo "  → Resource limits already present, skipping"
        continue
    fi
    
    # Add resource limits before [Install] section
    sed -i '/^\[Install\]/i \
# Resource limits for fault isolation\
CPUQuota=30%\
MemoryLimit=500M\
TasksMax=50\
' "$SERVICE_FILE"
    
    echo "  ✅ Resource limits added"
done

# Reload systemd
echo ""
echo "Reloading systemd daemon..."
systemctl daemon-reload

echo ""
echo "✅ Complete! Resource limits added to WebUI services."
echo ""
echo "Limits applied:"
echo "  - CPUQuota: 30% (prevents runaway CPU usage)"
echo "  - MemoryLimit: 500M (prevents memory leaks)"
echo "  - TasksMax: 50 (prevents fork bombs)"
echo ""
echo "Restart services to apply:"
echo "  sudo systemctl restart r2d2-web-dashboard"
echo "  sudo systemctl restart r2d2-rosbridge"

