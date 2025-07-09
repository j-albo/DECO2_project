#!/bin/bash

# run_fast_livo2.sh - Run script for FAST-LIVO2 Docker container

set -e

echo "🚀 Starting FAST-LIVO2 with Livox Mid-360"
echo "========================================="

# Configuration
IMAGE_NAME="fast-livo2-mid360:latest"
CONTAINER_NAME="fast-livo2-container"

# Check if image exists
if ! docker image inspect ${IMAGE_NAME} >/dev/null 2>&1; then
    echo "❌ Docker image ${IMAGE_NAME} not found!"
    echo "Please build it first with: ./build_fast_livo2.sh"
    exit 1
fi

# Check if container is already running
if docker ps | grep -q ${CONTAINER_NAME}; then
    echo "⚠️  Container ${CONTAINER_NAME} is already running."
    echo "Would you like to:"
    echo "1) Attach to existing container"
    echo "2) Stop and restart container"
    echo "3) Exit"
    read -p "Choose option (1-3): " choice
    
    case $choice in
        1)
            echo "📡 Attaching to existing container..."
            docker exec -it ${CONTAINER_NAME} /bin/bash
            exit 0
            ;;
        2)
            echo "🛑 Stopping existing container..."
            docker stop ${CONTAINER_NAME}
            ;;
        3)
            echo "👋 Exiting..."
            exit 0
            ;;
        *)
            echo "❌ Invalid option. Exiting..."
            exit 1
            ;;
    esac
fi

# Prepare X11 forwarding
echo "🖥️  Preparing X11 forwarding..."
xhost +local:docker 2>/dev/null || echo "⚠️  Could not configure X11 forwarding"

# Check for USB devices
echo "🔍 Checking for USB devices..."
if [ -e /dev/video0 ]; then
    echo "✅ Found camera at /dev/video0"
    CAMERA_DEVICE="--device=/dev/video0:/dev/video0"
else
    echo "⚠️  No camera found at /dev/video0"
    CAMERA_DEVICE=""
fi

# Additional USB devices for ArduCam
if [ -e /dev/video1 ]; then
    echo "✅ Found additional camera at /dev/video1"
    CAMERA_DEVICE="${CAMERA_DEVICE} --device=/dev/video1:/dev/video1"
fi

# Run the container
echo "🚀 Starting container..."
docker run -it --rm \
    --name ${CONTAINER_NAME} \
    --privileged \
    --net=host \
    -e DISPLAY=${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/data":/home/developer/data:rw \
    -v "$(pwd)/config":/home/developer/config:rw \
    ${CAMERA_DEVICE} \
    ${IMAGE_NAME}

# Cleanup X11 forwarding
echo "🧹 Cleaning up X11 forwarding..."
xhost -local:docker 2>/dev/null || true

echo "👋 Container stopped."
