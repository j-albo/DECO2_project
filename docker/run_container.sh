#!/bin/bash

# FAST-LIVO2 Docker Container Run Script
# This script starts the FAST-LIVO2 container with proper GUI support

set -e  # Exit on any error

echo "Starting FAST-LIVO2 container..."

# Check if the image exists
if ! docker image inspect fast-livo2:latest >/dev/null 2>&1; then
    echo "Error: FAST-LIVO2 Docker image not found."
    echo "Please build the image first by running: ./docker/build_image.sh"
    exit 1
fi

# Configure X11 forwarding for GUI applications (Linux only)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Configuring GUI support..."
    xhost +local:docker
    X11_FORWARDING="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
else
    echo "Warning: GUI support may not work on this platform."
    echo "rviz and other GUI applications may not display properly."
    X11_FORWARDING=""
fi

# Get script directory and move to parent
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Create data directories if they don't exist
mkdir -p "$PROJECT_DIR/data"
mkdir -p "$PROJECT_DIR/bags"
mkdir -p "$PROJECT_DIR/config"

echo "Data directories:"
echo "  Host data: $PROJECT_DIR/data -> Container: /home/developer/data"
echo "  Host bags: $PROJECT_DIR/bags -> Container: /home/developer/bags"
echo "  Host config: $PROJECT_DIR/config -> Container: /home/developer/config"
echo ""

echo "Available commands inside container:"
echo "  roslaunch fast_livo mapping_avia.launch"
echo "  rosbag play /home/developer/bags/YOUR_FILE.bag"
echo "  rviz"
echo ""

echo "To open additional terminals:"
echo "  docker exec -it fast-livo2-container bash"
echo ""

# Run the container
docker run -it \
    --name fast-livo2-container \
    --rm \
    --privileged \
    --net=host \
    $X11_FORWARDING \
    -v "$PROJECT_DIR/data":/home/developer/data:rw \
    -v "$PROJECT_DIR/bags":/home/developer/bags:rw \
    -v "$PROJECT_DIR/config":/home/developer/config:rw \
    --workdir /home/developer \
    fast-livo2:latest

# Restore X11 permissions on Linux
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Restoring X11 permissions..."
    xhost -local:docker
fi

echo "Container exited successfully."
