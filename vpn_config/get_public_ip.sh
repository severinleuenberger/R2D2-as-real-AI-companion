#!/bin/bash
# Get public IP address for WireGuard configuration

echo "Fetching public IP address..."
PUBLIC_IP=$(curl -s ifconfig.me || curl -s icanhazip.com || curl -s ipinfo.io/ip)

if [ -z "$PUBLIC_IP" ]; then
    echo "ERROR: Could not determine public IP address"
    echo "Please check your internet connection or set it manually"
    exit 1
fi

echo ""
echo "Your public IP address is: $PUBLIC_IP"
echo ""
echo "Update client_wg0.conf with this IP:"
echo "  Replace 'YOUR_PUBLIC_IP' with: $PUBLIC_IP"
echo ""

