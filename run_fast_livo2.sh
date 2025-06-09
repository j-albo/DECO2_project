#!/bin/bash

# FAST-LIVO2 Automated Runner
# Script for completely automated execution of FAST-LIVO2
# Author: FAST-LIVO2 automation script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
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
    echo "FAST-LIVO2 Automated Runner"
    echo ""
    echo "Usage: $0 [options] [bagfile.bag]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help"
    echo "  -b, --build             Build Docker image before running"
    echo "  -c, --config FILE       Use specific configuration file"
    echo "  -l, --launch FILE       Use specific launch file (default: mapping_avia.launch)"
    echo "  -d, --delay SECONDS     Delay before starting rosbag (default: 10)"
    echo "  --no-rviz               Don't open rviz automatically"
    echo "  --list-bags             List available .bag files"
    echo ""
    echo "Examples:"
    echo "  $0                              # Run with interactive selector"
    echo "  $0 my_dataset.bag               # Run with specific file"
    echo "  $0 -c calibration.yaml data.bag # Run with specific configuration"
    echo "  $0 --build --list-bags          # Build image and list bags"
    echo ""
}

# Function to list available .bag files
list_bags() {
    print_status "Available .bag files in bags/ directory:"
    if [ -d "bags" ]; then
        find bags/ -name "*.bag" -type f | sed 's|bags/||' | sort
        echo ""
        echo "Total: $(find bags/ -name "*.bag" -type f | wc -l) files"
    else
        print_warning "bags/ directory does not exist"
    fi
}

# Function for interactive bag file selector
select_bag_interactive() {
    if [ ! -d "bags" ]; then
        print_error "bags/ directory does not exist. Create it and place .bag files there."
        exit 1
    fi

    # Search for .bag files
    mapfile -t bag_files < <(find bags/ -name "*.bag" -type f | sed 's|bags/||' | sort)
    
    if [ ${#bag_files[@]} -eq 0 ]; then
        print_error "No .bag files found in bags/ directory"
        print_status "Place .bag files in the bags/ directory and try again"
        exit 1
    fi

    echo ""
    print_status "Available .bag files:"
    for i in "${!bag_files[@]}"; do
        echo "  $((i+1))) ${bag_files[$i]}"
    done
    echo ""

    while true; do
        read -p "Select a file (1-${#bag_files[@]}): " choice
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#bag_files[@]} ]; then
            selected_bag="${bag_files[$((choice-1))]}"
            break
        else
            print_error "Invalid selection. Enter a number between 1 and ${#bag_files[@]}"
        fi
    done
}

# Function to check dependencies
check_dependencies() {
    print_status "Checking dependencies..."
    
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "Docker is not running or you don't have permissions"
        exit 1
    fi
    
    print_success "Dependencies verified"
}

# Function to build image if necessary
ensure_image() {
    if [ "$BUILD_IMAGE" = true ]; then
        print_status "Building Docker image..."
        if [ -f "docker/build_image.sh" ]; then
            chmod +x docker/build_image.sh
            ./docker/build_image.sh
        else
            print_error "build_image.sh script not found"
            exit 1
        fi
    else
        # Check if image exists
        if ! docker image inspect fast-livo2:latest >/dev/null 2>&1; then
            print_warning "fast-livo2:latest image not found"
            print_status "Building image automatically..."
            if [ -f "docker/build_image.sh" ]; then
                chmod +x docker/build_image.sh
                ./docker/build_image.sh
            else
                print_error "build_image.sh script not found"
                exit 1
            fi
        fi
    fi
}

# Function to configure GUI
setup_gui() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        print_status "Configuring GUI support for Linux..."
        xhost +local:docker
        X11_FORWARDING="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    else
        print_warning "GUI support may not work on this platform"
        X11_FORWARDING=""
    fi
}

# Function to create necessary directories
setup_directories() {
    print_status "Setting up directories..."
    mkdir -p data bags config
    print_success "Directories configured"
}

# Function to clean up previous containers
cleanup_containers() {
    print_status "Cleaning up previous containers..."
    docker stop fast-livo2-container 2>/dev/null || true
    docker rm fast-livo2-container 2>/dev/null || true
}

# Function to run FAST-LIVO2
run_fast_livo2() {
    local bag_file="$1"
    local config_file="$2"
    local launch_file="$3"
    local delay="$4"
    local no_rviz="$5"
    
    print_status "Starting FAST-LIVO2..."
    print_status "Bag file: $bag_file"
    print_status "Configuration: ${config_file:-"default"}"
    print_status "Launch file: $launch_file"
    print_status "Delay before rosbag: ${delay}s"
    
    # Configure configuration arguments
    local config_args=""
    if [ -n "$config_file" ]; then
        if [ -f "config/$config_file" ]; then
            config_args="config_file:=/home/developer/config/$config_file"
        elif [ -f "bags/$config_file" ]; then
            config_args="config_file:=/home/developer/bags/$config_file"
        else
            print_error "Configuration file not found: $config_file"
            exit 1
        fi
    fi

    # Create automation script inside container
    cat > /tmp/fast_livo2_auto.sh << EOF
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/developer/dev_ws/devel/setup.bash

echo "=== FAST-LIVO2 Automated Execution ==="
echo "Bag file: $bag_file"
echo "Config: ${config_file:-"default"}"
echo "Launch: $launch_file"
echo ""

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

# Check if roscore is already running
if ! rostopic list &>/dev/null; then
    echo "🚀 Starting ROS Master..."
    roscore &
    ROSCORE_PID=\$!
    sleep 3
    
    # Verify roscore is working
    if ! rostopic list &>/dev/null; then
        echo "❌ Error: roscore is not working"
        exit 1
    fi
    echo "✅ ROS Master started"
else
    echo "✅ ROS Master already running"
fi

# Launch FAST-LIVO2
echo "🚀 Starting FAST-LIVO2..."
roslaunch fast_livo $launch_file $config_args &
FASTLIVO_PID=\$!
sleep $delay

# Verify FAST-LIVO2 is working
if ! ps -p \$FASTLIVO_PID > /dev/null; then
    echo "❌ Error: FAST-LIVO2 failed to start"
    exit 1
fi

echo "✅ FAST-LIVO2 started successfully"

# Launch rviz ONLY ONCE if enabled
if [ "$no_rviz" != "true" ]; then
    # Check that no other rviz is running
    if ! pgrep -f rviz > /dev/null; then
        echo "📊 Starting rviz..."
        rviz &
        RVIZ_PID=\$!
        sleep 5
        
        # Verify rviz started correctly
        if ps -p \$RVIZ_PID > /dev/null; then
            echo "✅ rviz started successfully"
        else
            echo "⚠️  rviz could not start (may be GUI issue)"
        fi
    else
        echo "✅ rviz already running"
    fi
fi

# Wait a bit more for everything to stabilize
sleep 3

# Play rosbag
echo "📁 Playing dataset: $bag_file"
echo "⏯️  Starting playback in 3 seconds..."
sleep 3

rosbag play /home/developer/bags/$bag_file --clock &
ROSBAG_PID=\$!

echo ""
echo "=== FAST-LIVO2 Running ==="
echo "📊 Open rviz manually if it didn't open automatically: rviz"
echo "🔧 To configure rviz:"
echo "   - Fixed Frame: camera_init"
echo "   - Add PointCloud2: /cloud_registered"
echo "   - Add Path: /path"
echo "   - Add Image: /left_camera/image"
echo ""
echo "⏹️  Press Ctrl+C to stop all processes"
echo ""

# Wait for rosbag to finish or be interrupted
wait \$ROSBAG_PID

echo "✅ Dataset playback completed"
echo "🔄 FAST-LIVO2 processes still running..."
echo "⏹️  Press Ctrl+C to exit completely"

# Keep processes active until user decides to exit
while true; do
    sleep 1
done
EOF

    chmod +x /tmp/fast_livo2_auto.sh

    # Run container with automated script
    print_status "Running Docker container..."
    docker run -it \
        --name fast-livo2-container \
        --rm \
        --privileged \
        --net=host \
        $X11_FORWARDING \
        -v "$(pwd)/data":/home/developer/data:rw \
        -v "$(pwd)/bags":/home/developer/bags:rw \
        -v "$(pwd)/config":/home/developer/config:rw \
        -v /tmp/fast_livo2_auto.sh:/home/developer/auto_run.sh:ro \
        --workdir /home/developer \
        fast-livo2:latest \
        /home/developer/auto_run.sh
}

# Cleanup function on exit
cleanup_on_exit() {
    print_status "Cleaning up..."
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        xhost -local:docker 2>/dev/null || true
    fi
    cleanup_containers
    rm -f /tmp/fast_livo2_auto.sh
}

# Configure trap to cleanup on exit
trap cleanup_on_exit EXIT

# Default variables
BUILD_IMAGE=false
CONFIG_FILE=""
LAUNCH_FILE="mapping_avia.launch"
DELAY=10
NO_RVIZ=false
SELECTED_BAG=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -b|--build)
            BUILD_IMAGE=true
            shift
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -l|--launch)
            LAUNCH_FILE="$2"
            shift 2
            ;;
        -d|--delay)
            DELAY="$2"
            shift 2
            ;;
        --no-rviz)
            NO_RVIZ=true
            shift
            ;;
        --list-bags)
            list_bags
            exit 0
            ;;
        -*)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
        *)
            if [ -z "$SELECTED_BAG" ]; then
                SELECTED_BAG="$1"
            else
                print_error "Too many arguments"
                show_help
                exit 1
            fi
            shift
            ;;
    esac
done

# Main function
main() {
    echo "🚀 FAST-LIVO2 Automated Runner"
    echo "================================"
    echo ""
    
    # Check dependencies
    check_dependencies
    
    # Setup directories
    setup_directories
    
    # Build image if necessary
    ensure_image
    
    # Select .bag file
    if [ -z "$SELECTED_BAG" ]; then
        select_bag_interactive
    else
        selected_bag="$SELECTED_BAG"
        # Verify file exists
        if [ ! -f "bags/$selected_bag" ]; then
            print_error "File not found: bags/$selected_bag"
            exit 1
        fi
    fi
    
    print_success "Selected file: $selected_bag"
    
    # Configure GUI
    setup_gui
    
    # Clean previous containers
    cleanup_containers
    
    # Run FAST-LIVO2
    run_fast_livo2 "$selected_bag" "$CONFIG_FILE" "$LAUNCH_FILE" "$DELAY" "$NO_RVIZ"
}

# Execute main function
main "$@"