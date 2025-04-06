#!/bin/bash

# Build the Docker image
echo "Building signaling server Docker image..."
docker build -t signaling-server -f Dockerfile.signaling .

# Run the container
echo "Starting signaling server container..."
docker run -d --name signaling-test -p 8765:8765 signaling-server

# Wait for server to start
echo "Waiting for server to start..."
sleep 2

# Test the server using websocat
echo "Testing WebSocket connection..."
echo "Type your message and press Enter to send. Press Ctrl+C to exit."
echo "Example message: {\"type\": \"test\", \"data\": \"hello\"}"
echo "----------------------------------------"
echo "WebSocket connection open. Type your messages now:"

# Run websocat interactively (without piping anything to it)
websocat ws://localhost:8765

# Cleanup
echo "Cleaning up..."
docker stop signaling-test
docker rm signaling-test 