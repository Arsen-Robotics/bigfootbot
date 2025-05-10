FROM python:3.9-slim

# Install required dependencies
RUN apt-get update && apt-get install -y \
    libavdevice-dev \
    libavfilter-dev \
    libopus-dev \
    libvpx-dev \
    pkg-config \
    libsrtp2-dev \
    libx264-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code and static files
COPY src/ ./src/
COPY static/ ./static/

# Expose port for HTTP
EXPOSE 8080

# Run the server
CMD ["python", "src/main.py"] 