#!/bin/bash

# build_fast_livo2.sh - Build script for FAST-LIVO2 Docker image

set -e

echo "🚀 Building FAST-LIVO2 with Livox Mid-360 Docker Image"
echo "======================================================"

# Configuration
IMAGE_NAME="fast-livo2-mid360"
IMAGE_TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}"

# Build the image
echo "📦 Building Docker image: ${FULL_IMAGE_NAME}"
docker build -t ${FULL_IMAGE_NAME} .

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "🎯 To run the container:"
    echo "docker run -it --rm \\"
    echo "  --name fast-livo2-container \\"
    echo "  --privileged \\"
    echo "  --net=host \\"
    echo "  -e DISPLAY=\$DISPLAY \\"
    echo "  -e QT_X11_NO_MITSHM=1 \\"
    echo "  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
    echo "  --device=/dev/video0:/dev/video0 \\"
    echo "  ${FULL_IMAGE_NAME}"
    echo ""
    echo "🚀 Or use the convenience script:"
    echo "./run_fast_livo2.sh"
else
    echo "❌ Build failed!"
    exit 1
fi
