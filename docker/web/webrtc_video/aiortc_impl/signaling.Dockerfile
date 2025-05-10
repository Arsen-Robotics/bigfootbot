FROM python:3.9-slim

# Install required dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the signaling server code
COPY src/signaling_server.py ./src/

# Expose port for WebSocket
EXPOSE 8765

# Run the signaling server
CMD ["python", "src/signaling_server.py"] 