#!/bin/bash
# Diagnose and fix VPN setup issues
# Run with: sudo bash diagnose_and_fix.sh

set -e

echo "=== WireGuard VPN Diagnosis and Fix ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "[1/6] Checking WireGuard installation..."
if command -v wg &> /dev/null; then
    echo "   ✅ WireGuard is installed: $(wg --version)"
else
    echo "   ❌ WireGuard not installed - installing now..."
    apt update
    apt install -y wireguard wireguard-tools
fi

echo "[2/6] Checking /etc/wireguard directory..."
if [ ! -d /etc/wireguard ]; then
    echo "   ⚠️  Directory missing - creating..."
    mkdir -p /etc/wireguard
fi

echo "[3/6] Checking for existing keys..."
if [ ! -f /etc/wireguard/server_private.key ] || [ ! -f /etc/wireguard/client_private.key ]; then
    echo "   ⚠️  Keys missing - generating new keys..."
    cd /etc/wireguard
    
    # Generate server keys
    if [ ! -f server_private.key ]; then
        wg genkey | tee server_private.key | wg pubkey > server_public.key
        chmod 600 server_private.key
        chmod 644 server_public.key
        echo "   ✅ Server keys generated"
    fi
    
    # Generate client keys
    if [ ! -f client_private.key ]; then
        wg genkey | tee client_private.key | wg pubkey > client_public.key
        chmod 600 client_private.key
        chmod 644 client_public.key
        echo "   ✅ Client keys generated"
    fi
else
    echo "   ✅ Keys exist"
fi

echo "[4/6] Checking server configuration..."
if [ ! -f /etc/wireguard/wg0.conf ]; then
    echo "   ⚠️  Server config missing - creating..."
    
    # Get keys
    SERVER_PRIVATE_KEY=$(cat /etc/wireguard/server_private.key)
    SERVER_PUBLIC_KEY=$(cat /etc/wireguard/server_public.key)
    CLIENT_PUBLIC_KEY=$(cat /etc/wireguard/client_public.key)
    
    # Get network interface
    INTERFACE=$(ip route | grep default | awk '{print $5}' | head -1)
    SERVER_IP=$(ip -4 addr show $INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1)
    
    # VPN network
    VPN_SERVER_IP="10.8.0.1"
    VPN_CLIENT_IP="10.8.0.2"
    VPN_PORT="51820"
    
    # Create server config
    cat > /etc/wireguard/wg0.conf <<EOF
[Interface]
# Server configuration
Address = $VPN_SERVER_IP/24
ListenPort = $VPN_PORT
PrivateKey = $SERVER_PRIVATE_KEY

# Enable NAT forwarding
PostUp = iptables -A FORWARD -i wg0 -j ACCEPT; iptables -A FORWARD -o wg0 -j ACCEPT; iptables -t nat -A POSTROUTING -o $INTERFACE -j MASQUERADE
PostDown = iptables -D FORWARD -i wg0 -j ACCEPT; iptables -D FORWARD -o wg0 -j ACCEPT; iptables -t nat -D POSTROUTING -o $INTERFACE -j MASQUERADE

[Peer]
# Client configuration
PublicKey = $CLIENT_PUBLIC_KEY
AllowedIPs = $VPN_CLIENT_IP/32
EOF
    
    chmod 600 /etc/wireguard/wg0.conf
    echo "   ✅ Server config created"
else
    echo "   ✅ Server config exists"
fi

echo "[5/6] Checking client configuration..."
if [ ! -f /home/severin/dev/r2d2/vpn_config/client_wg0.conf ]; then
    echo "   ⚠️  Client config missing - creating..."
    
    CLIENT_PRIVATE_KEY=$(cat /etc/wireguard/client_private.key)
    SERVER_PUBLIC_KEY=$(cat /etc/wireguard/server_public.key)
    VPN_CLIENT_IP="10.8.0.2"
    VPN_PORT="51820"
    
    mkdir -p /home/severin/dev/r2d2/vpn_config
    
    cat > /home/severin/dev/r2d2/vpn_config/client_wg0.conf <<EOF
[Interface]
# Client configuration
PrivateKey = $CLIENT_PRIVATE_KEY
Address = $VPN_CLIENT_IP/24
DNS = 8.8.8.8

[Peer]
# Server configuration
PublicKey = $SERVER_PUBLIC_KEY
Endpoint = YOUR_PUBLIC_IP:$VPN_PORT
AllowedIPs = 0.0.0.0/0
PersistentKeepalive = 25
EOF
    
    chmod 600 /home/severin/dev/r2d2/vpn_config/client_wg0.conf
    chown severin:severin /home/severin/dev/r2d2/vpn_config/client_wg0.conf
    echo "   ✅ Client config created"
else
    echo "   ✅ Client config exists"
fi

echo "[6/6] Creating secure storage directory..."
SECURE_DIR="/home/severin/.r2d2_vpn_secrets"
mkdir -p "$SECURE_DIR"
chmod 700 "$SECURE_DIR"

# Save sensitive information
SERVER_PRIVATE_KEY=$(cat /etc/wireguard/server_private.key)
SERVER_PUBLIC_KEY=$(cat /etc/wireguard/server_public.key)
CLIENT_PRIVATE_KEY=$(cat /etc/wireguard/client_private.key)
CLIENT_PUBLIC_KEY=$(cat /etc/wireguard/client_public.key)
INTERFACE=$(ip route | grep default | awk '{print $5}' | head -1)
SERVER_IP=$(ip -4 addr show $INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1)

cat > "$SECURE_DIR/vpn_keys.txt" <<EOF
# R2D2 VPN - SENSITIVE INFORMATION
# DO NOT COMMIT TO GIT - STORE IN KEEPASS
# Generated: $(date)

=== SERVER KEYS ===
Server Private Key: $SERVER_PRIVATE_KEY
Server Public Key: $SERVER_PUBLIC_KEY

=== CLIENT KEYS ===
Client Private Key: $CLIENT_PRIVATE_KEY
Client Public Key: $CLIENT_PUBLIC_KEY

=== NETWORK CONFIGURATION ===
VPN Server IP: 10.8.0.1
VPN Client IP: 10.8.0.2
VPN Port: 51820
Jetson Local IP: $SERVER_IP
Network Interface: $INTERFACE

=== CLIENT CONFIG FILE ===
Location: /home/severin/dev/r2d2/vpn_config/client_wg0.conf
(Update YOUR_PUBLIC_IP before using)
EOF

chmod 600 "$SECURE_DIR/vpn_keys.txt"
chown severin:severin "$SECURE_DIR/vpn_keys.txt"

# Copy client config backup
cp /home/severin/dev/r2d2/vpn_config/client_wg0.conf "$SECURE_DIR/client_wg0.conf.backup" 2>/dev/null || true
chmod 600 "$SECURE_DIR/client_wg0.conf.backup" 2>/dev/null || true
chown severin:severin "$SECURE_DIR/client_wg0.conf.backup" 2>/dev/null || true

echo "   ✅ Secure storage created"

# Enable IP forwarding
echo ""
echo "[BONUS] Enabling IP forwarding..."
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf; then
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
    sysctl -p
    echo "   ✅ IP forwarding enabled"
else
    echo "   ✅ IP forwarding already enabled"
fi

echo ""
echo "=== Diagnosis and Fix Complete! ==="
echo ""
echo "✅ All components are now in place"
echo ""
echo "Server Public Key: $SERVER_PUBLIC_KEY"
echo "Client Public Key: $CLIENT_PUBLIC_KEY"
echo ""
echo "Sensitive information saved to: $SECURE_DIR/vpn_keys.txt"
echo ""
echo "Next steps:"
echo "1. Update client config with public IP:"
echo "   sudo sed -i 's/YOUR_PUBLIC_IP/188.61.209.189/g' /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo ""
echo "2. Start WireGuard service:"
echo "   sudo systemctl enable wg-quick@wg0"
echo "   sudo systemctl start wg-quick@wg0"
echo ""
echo "3. Verify:"
echo "   sudo systemctl status wg-quick@wg0"
echo "   sudo wg show"
echo ""

