FROM python:3.10-slim

WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY src /app/src
COPY static /app/static

# Expose the HTTP port
EXPOSE 8080

# Run the HTTP server
CMD ["python", "src/http_server.py"] 