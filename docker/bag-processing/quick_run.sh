#!/bin/bash

# FAST-LIVO2 Quick Runner - Simplified Version
# Runs FAST-LIVO2 with the first .bag file found

set -e

echo "🚀 FAST-LIVO2 Quick Runner"
echo "=========================="

# Check Docker
if ! command -v docker &> /dev/null; then
    echo "❌ Error: Docker is not installed"
    exit 1
fi

# Create directories if they don't exist
mkdir -p bags data config

# Search for .bag files
bag_files=(bags/*.bag)
if [ ! -f "${bag_files[0]}" ]; then
    echo "❌ No .bag files found in bags/ directory"
    echo "📁 Place .bag files in the bags/ directory and try again"
    exit 1
fi

# Use the first file found
selected_bag=$(basename "${bag_files[0]}")
echo "📁 Using file: $selected_bag"

# Check Docker image
if ! docker image inspect fast-livo2:latest >/dev/null 2>&1; then
    echo "🔨 Building Docker image..."
    if [ -f "docker/build_image.sh" ]; then
        chmod +x docker/build_image.sh
        ./docker/build_image.sh
    else
        echo "❌ build_image.sh script not found"
        exit 1
    fi
fi

# Configure GUI for Linux
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xhost +local:docker
    X11_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
else
    X11_ARGS=""
fi

# Clean previous containers
docker stop fast-livo2-container 2>/dev/null || true
docker rm fast-livo2-container 2>/dev/null || true

echo "🚀 Starting FAST-LIVO2..."
echo "⏱️  Waiting 15 seconds before playing dataset..."

# Create simple script for container
cat > /tmp/simple_run.sh << 'EOF'
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/developer/dev_ws/devel/setup.bash

# Cleanup function
cleanup() {
    echo "Cleaning up processes..."
    pkill -f roslaunch || true
    pkill -f rosbag || true
    pkill -f rviz || true
    pkill -f roscore || true
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🚀 Starting ROS Master..."
if ! rostopic list &>/dev/null; then
    roscore &
    sleep 3
fi

echo "🚀 Starting FAST-LIVO2..."
roslaunch fast_livo mapping_avia.launch &
sleep 15

echo "📊 Starting rviz (single instance only)..."
if ! pgrep -f rviz > /dev/null; then
    rviz &
    sleep 5
fi

echo "📁 Playing dataset: $1"
rosbag play /home/developer/bags/$1 --clock

echo "✅ Dataset played! Processes remain active."
echo "⏹️  Press Ctrl+C to exit completely."

# Keep processes active
while true; do
    sleep 1
done
EOF

chmod +x /tmp/simple_run.sh

# Run container
docker run -it \
    --name fast-livo2-container \
    --rm \
    --privileged \
    --net=host \
    $X11_ARGS \
    -v "$(pwd)/data":/home/developer/data:rw \
    -v "$(pwd)/bags":/home/developer/bags:rw \
    -v "$(pwd)/config":/home/developer/config:rw \
    -v /tmp/simple_run.sh:/tmp/run.sh:ro \
    --workdir /home/developer \
    fast-livo2:latest \
    /tmp/run.sh "$selected_bag"

# Cleanup
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xhost -local:docker
fi

rm -f /tmp/simple_run.sh
echo "✅ Completed!"