#!/bin/bash

# Get the local IP address
LOCAL_IP=$(hostname -I | awk '{print $1}')

# Find the public IP address
PUBLIC_IP=$(curl -s https://api.ipify.org)

# Print setup information
echo "====================================================="
echo "WebRTC Camera Remote Access Setup"
echo "====================================================="
echo ""
echo "LOCAL NETWORK INFORMATION:"
echo "  - Local IP address: $LOCAL_IP"
echo "  - Public IP address: $PUBLIC_IP"
echo ""
echo "REQUIRED ROUTER CONFIGURATION:"
echo "1. Log into your router administration panel"
echo "2. Find the 'Port Forwarding' section"
echo "3. Add these port forwarding rules:"
echo "   - Forward TCP/UDP port 8080 to $LOCAL_IP:8080 (Web Server)"
echo "   - Forward TCP/UDP port 8765 to $LOCAL_IP:8765 (Signaling Server)"
echo ""
echo "ACCESSING FROM OUTSIDE YOUR NETWORK:"
echo "1. From your smartphone browser (on LTE/cellular):"
echo "   - Access: http://$PUBLIC_IP:8080"
echo "2. Enter server address in the app: $PUBLIC_IP:8765"
echo ""
echo "NOTE: Your public IP may change if you don't have a static IP"
echo "Consider using a dynamic DNS service like:"
echo "  - No-IP (noip.com)"
echo "  - DuckDNS (duckdns.org)"
echo "  - Dynu (dynu.com)"
echo ""
echo "=====================================================" 