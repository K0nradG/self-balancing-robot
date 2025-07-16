FROM python:3.10-slim

# 1. Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 2. Install the latest available stable version of nrfutil (6.1.7)
RUN pip install --no-cache-dir nrfutil==6.1.7

# 3. Optional: Install additional tools for debugging (e.g., lsusb)
RUN apt-get update && apt-get install -y --no-install-recommends \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 4. Set working directory inside the container
WORKDIR /app
