#!/bin/bash

echo "🚀 Building Docker image for FAST-LIVO2..."
echo "This may take 20-30 minutes the first time..."

# Go to Dockerfile directory
cd "$(dirname "$0")"

# Build the image
docker build -t fast-livo2:latest .

if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully!"
    echo "To run: ./docker/run_container.sh"
else
    echo "❌ Build error"
    exit 1
fi
