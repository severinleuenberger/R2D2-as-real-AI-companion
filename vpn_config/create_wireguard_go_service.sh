#!/bin/bash
# Create a systemd service that uses wireguard-go directly
# Run with: sudo bash create_wireguard_go_service.sh

set -e

echo "=== Creating WireGuard-Go Systemd Service ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

CONFIG_FILE="/etc/wireguard/wg0.conf"
SERVICE_FILE="/etc/systemd/system/wireguard-go@wg0.service"

echo "[1/3] Checking configuration..."
if [ ! -f "$CONFIG_FILE" ]; then
    echo "   ❌ Config file not found: $CONFIG_FILE"
    exit 1
fi
echo "   ✅ Config file exists"

echo "[2/3] Creating systemd service for wireguard-go..."
cat > "$SERVICE_FILE" <<'EOF'
[Unit]
Description=WireGuard VPN (userspace via wireguard-go) for %i
After=network.target
Before=network-online.target
Wants=network-online.target

[Service]
Type=notify
ExecStartPre=/bin/bash -c 'modprobe tun || true'
ExecStart=/usr/bin/wireguard-go %i
ExecStartPost=/bin/bash -c 'sleep 1 && /usr/bin/wg setconf %i /etc/wireguard/%i.conf'
ExecStop=/bin/bash -c '/usr/bin/wg-quick down %i || /usr/bin/ip link delete %i || true'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "   ✅ Service file created"

echo "[3/3] Creating wrapper script..."
# Create a wrapper that uses wireguard-go with TUN
cat > /usr/local/bin/wg-quick-go <<'EOF'
#!/bin/bash
# Wrapper to use wireguard-go instead of kernel module

INTERFACE="$1"
ACTION="$2"
CONFIG="/etc/wireguard/${INTERFACE}.conf"

if [ "$ACTION" != "up" ] && [ "$ACTION" != "down" ]; then
    echo "Usage: $0 <interface> [up|down]"
    exit 1
fi

if [ "$ACTION" = "up" ]; then
    # Load TUN module
    modprobe tun 2>/dev/null || true
    
    # Start wireguard-go in background
    wireguard-go "$INTERFACE" &
    WG_GO_PID=$!
    sleep 2
    
    # Apply configuration
    wg setconf "$INTERFACE" "$CONFIG"
    
    # Set IP address from config
    IP=$(grep "^Address" "$CONFIG" | head -1 | awk '{print $3}' | cut -d'/' -f1)
    if [ -n "$IP" ]; then
        ip addr add "$IP/24" dev "$INTERFACE" 2>/dev/null || true
        ip link set "$INTERFACE" up
    fi
    
    echo "$WG_GO_PID" > /var/run/wireguard-go-${INTERFACE}.pid
elif [ "$ACTION" = "down" ]; then
    # Stop wireguard-go
    if [ -f "/var/run/wireguard-go-${INTERFACE}.pid" ]; then
        kill $(cat /var/run/wireguard-go-${INTERFACE}.pid) 2>/dev/null || true
        rm -f /var/run/wireguard-go-${INTERFACE}.pid
    fi
    ip link delete "$INTERFACE" 2>/dev/null || true
fi
EOF

chmod +x /usr/local/bin/wg-quick-go

echo "   ✅ Wrapper script created"

# Disable the old service
systemctl disable wg-quick@wg0 2>/dev/null || true

# Enable new service
systemctl daemon-reload
systemctl enable wireguard-go@wg0

echo ""
echo "✅ Setup complete!"
echo ""
echo "Now try starting the service:"
echo "   sudo systemctl start wireguard-go@wg0"
echo "   sudo systemctl status wireguard-go@wg0"
echo ""

