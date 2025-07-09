#!/bin/bash

# FAST-LIVO2 Real-time Runner
# Script for real-time operation with Livox Mid-360 + ArduCam

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show help
show_help() {
    echo "FAST-LIVO2 Real-time Runner"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help"
    echo "  -c, --config FILE       Use specific configuration file"
    echo "  --rviz                  Launch rviz for visualization"
    echo "  --record DURATION       Record rosbag for DURATION seconds"
    echo "  --calibrate             Launch calibration mode"
    echo "  --check-devices         Check connected devices"
    echo ""
    echo "Examples:"
    echo "  $0                      # Run real-time FAST-LIVO2"
    echo "  $0 --rviz              # Run with rviz visualization"
    echo "  $0 --record 60         # Run and record 60 seconds"
    echo "  $0 --check-devices     # Check Livox and camera"
    echo ""
}

# Function to check devices
check_devices() {
    print_status "Checking connected devices..."
    
    # Check USB devices
    print_status "USB devices:"
    lsusb | grep -E "(Livox|Camera|ArduCam)" || print_warning "No Livox/ArduCam devices found via USB"
    
    # Check video devices
    print_status "Video devices:"
    ls -la /dev/video* 2>/dev/null || print_warning "No video devices found"
    
    # Check V4L2 devices
    if command -v v4l2-ctl &> /dev/null; then
        print_status "Camera capabilities:"
        v4l2-ctl --list-devices 2>/dev/null || print_warning "No V4L2 devices found"
    fi
    
    # Check network interfaces (for Livox)
    print_status "Network interfaces:"
    ip addr | grep -E "(192\.168\.1\.|eth|wlan)" || print_warning "Check network configuration for Livox"
    
    print_success "Device check completed"
}

# Function to setup real-time permissions
setup_realtime() {
    print_status "Setting up real-time permissions..."
    
    # Set CPU governor to performance
    echo "Setting CPU governor to performance..."
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null 2>&1 || true
    
    # Set process priority
    renice -n -10 $$ > /dev/null 2>&1 || true
    
    print_success "Real-time setup completed"
}

# Function to run calibration
run_calibration() {
    print_status "Starting calibration mode..."
    print_warning "Make sure your calibration target is visible to both LiDAR and camera"
    
    # Launch calibration-specific configuration
    roslaunch fast_livo calibration_mid360.launch
}

# Function to record data
record_data() {
    local duration=$1
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local bag_file="realtime_recording_${timestamp}.bag"
    
    print_status "Recording data for ${duration} seconds..."
    print_status "Output file: ${bag_file}"
    
    # Start recording in background
    rosbag record -O "${bag_file}" \
        /livox/lidar \
        /livox/imu \
        /camera/image_raw \
        /camera/camera_info \
        /fast_livo/odometry \
        /fast_livo/path \
        /fast_livo/cloud_registered \
        --duration="${duration}" &
    
    RECORD_PID=$!
    
    print_success "Recording started (PID: ${RECORD_PID})"
    return $RECORD_PID
}

# Function to run FAST-LIVO2 real-time
run_realtime() {
    local config_file="$1"
    local enable_rviz="$2"
    local record_duration="$3"
    
    print_status "Starting FAST-LIVO2 real-time processing..."
    print_status "Configuration: ${config_file:-"default"}"
    print_status "Visualization: ${enable_rviz}"
    
    # Setup real-time environment
    setup_realtime
    
    # Configure launch arguments
    local launch_args=""
    if [ -n "$config_file" ]; then
        launch_args="config_file:=${config_file}"
    fi
    
    if [ "$enable_rviz" = "true" ]; then
        launch_args="${launch_args} enable_rviz:=true"
    fi
    
    # Start recording if requested
    local record_pid=""
    if [ -n "$record_duration" ]; then
        record_data "$record_duration"
        record_pid=$!
    fi
    
    print_success "Launching FAST-LIVO2..."
    
    # Create cleanup function
    cleanup() {
        print_status "Shutting down..."
        if [ -n "$record_pid" ]; then
            kill $record_pid 2>/dev/null || true
            print_status "Recording stopped"
        fi
        pkill -f roslaunch 2>/dev/null || true
        pkill -f fastlivo_mapping 2>/dev/null || true
        exit 0
    }
    
    trap cleanup SIGINT SIGTERM
    
    # Launch FAST-LIVO2
    roslaunch fast_livo realtime_mid360.launch $launch_args
    
    print_success "FAST-LIVO2 real-time completed"
}

# Parse arguments
CONFIG_FILE=""
ENABLE_RVIZ=false
RECORD_DURATION=""
CHECK_DEVICES=false
CALIBRATE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --rviz)
            ENABLE_RVIZ=true
            shift
            ;;
        --record)
            RECORD_DURATION="$2"
            shift 2
            ;;
        --check-devices)
            CHECK_DEVICES=true
            shift
            ;;
        --calibrate)
            CALIBRATE=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Main execution
echo "🚀 FAST-LIVO2 Real-time Runner"
echo "=============================="
echo ""

# Check devices if requested
if [ "$CHECK_DEVICES" = true ]; then
    check_devices
    exit 0
fi

# Run calibration if requested
if [ "$CALIBRATE" = true ]; then
    run_calibration
    exit 0
fi

# Check if ROS is running
if ! pgrep -f roscore > /dev/null; then
    print_status "Starting ROS master..."
    roscore &
    sleep 3
fi

# Run real-time FAST-LIVO2
run_realtime "$CONFIG_FILE" "$ENABLE_RVIZ" "$RECORD_DURATION"
