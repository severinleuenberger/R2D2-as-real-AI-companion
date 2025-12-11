#!/bin/bash
# WireGuard VPN Setup Script for Jetson Orin
# Run with: sudo bash setup_wireguard.sh

set -e

echo "=== WireGuard VPN Setup for Jetson Orin ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Step 1: Install WireGuard
echo "[1/6] Installing WireGuard..."
apt update
apt install -y wireguard wireguard-tools

# Step 2: Enable IP forwarding
echo "[2/6] Enabling IP forwarding..."
echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
sysctl -p

# Step 3: Generate server keys
echo "[3/6] Generating server keys..."
mkdir -p /etc/wireguard
cd /etc/wireguard
wg genkey | tee server_private.key | wg pubkey > server_public.key
chmod 600 server_private.key
chmod 644 server_public.key

# Step 4: Generate client keys
echo "[4/6] Generating client keys..."
wg genkey | tee client_private.key | wg pubkey > client_public.key
chmod 600 client_private.key
chmod 644 client_public.key

# Step 5: Get network interface and IP
INTERFACE=$(ip route | grep default | awk '{print $5}' | head -1)
SERVER_IP=$(ip -4 addr show $INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1)

echo "[5/6] Detected network configuration:"
echo "  Interface: $INTERFACE"
echo "  Server IP: $SERVER_IP"

# Step 6: Create server configuration
echo "[6/6] Creating server configuration..."

SERVER_PRIVATE_KEY=$(cat /etc/wireguard/server_private.key)
SERVER_PUBLIC_KEY=$(cat /etc/wireguard/server_public.key)
CLIENT_PUBLIC_KEY=$(cat /etc/wireguard/client_public.key)

# Determine VPN network (use 10.8.0.0/24 to avoid conflicts)
VPN_SERVER_IP="10.8.0.1"
VPN_CLIENT_IP="10.8.0.2"
VPN_PORT="51820"

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

# Create client configuration file in home directory
CLIENT_PRIVATE_KEY=$(cat /etc/wireguard/client_private.key)

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

# Create secure storage directory (outside git repo, gitignored)
SECURE_DIR="$HOME/.r2d2_vpn_secrets"
mkdir -p "$SECURE_DIR"
chmod 700 "$SECURE_DIR"

# Save sensitive information to secure location
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
VPN Server IP: $VPN_SERVER_IP
VPN Client IP: $VPN_CLIENT_IP
VPN Port: $VPN_PORT
Jetson Local IP: $SERVER_IP
Network Interface: $INTERFACE

=== CLIENT CONFIG FILE ===
Location: /home/severin/dev/r2d2/vpn_config/client_wg0.conf
(Update YOUR_PUBLIC_IP before using)
EOF

chmod 600 "$SECURE_DIR/vpn_keys.txt"

# Copy client config to secure location as backup
cp /home/severin/dev/r2d2/vpn_config/client_wg0.conf "$SECURE_DIR/client_wg0.conf.backup"
chmod 600 "$SECURE_DIR/client_wg0.conf.backup"

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "⚠️  SECURITY: Sensitive information saved to: $SECURE_DIR/"
echo "   - vpn_keys.txt (all keys and config - STORE IN KEEPASS)"
echo "   - client_wg0.conf.backup (client config backup)"
echo ""
echo "Server Public Key: $SERVER_PUBLIC_KEY"
echo "Client Public Key: $CLIENT_PUBLIC_KEY"
echo ""
echo "📋 NEXT STEPS:"
echo "1. Copy sensitive info to KeePass (see $SECURE_DIR/vpn_keys.txt)"
echo "2. Edit client_wg0.conf and replace YOUR_PUBLIC_IP with your router's public IP"
echo "   Location: /home/severin/dev/r2d2/vpn_config/client_wg0.conf"
echo "3. Start WireGuard: sudo systemctl enable wg-quick@wg0 && sudo systemctl start wg-quick@wg0"
echo "4. Configure router port forwarding: UDP port $VPN_PORT -> $SERVER_IP:$VPN_PORT"
echo "5. Transfer client_wg0.conf to your Windows laptop (securely!)"
echo "6. Import into WireGuard client on Windows"
echo ""
echo "🔒 SECURITY REMINDER:"
echo "   - Never commit .key files or client_wg0.conf to git"
echo "   - Store all keys in KeePass"
echo "   - Delete client_wg0.conf from laptop after importing to WireGuard"
echo ""

