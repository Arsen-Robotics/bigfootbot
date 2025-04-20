#!/bin/bash

if [ "$1" = "signaling" ]; then
  exec /app/bin/signaling
elif [ "$1" = "sender" ]; then
  if [ -z "$2" ]; then
    echo "Error: Missing server address. Usage: sender <server_address> [<server_port>]"
    exit 1
  fi
  SERVER_ADDR=$2
  SERVER_PORT=${3:-8765}
  # List available video devices
  echo "Available video devices:"
  ls -la /dev/video* 2>/dev/null || echo "No video devices found"
  echo "Starting WebRTC sender connecting to $SERVER_ADDR:$SERVER_PORT"
  exec /app/bin/send_x86 -s ws://$SERVER_ADDR:$SERVER_PORT
elif [ "$1" = "web" ]; then
  exec /app/serve_client.sh
else
  echo "Usage: $0 {signaling|sender <server_address> [<server_port>]|web}"
  exit 1
fi 