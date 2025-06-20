# FAST-LIVO2 Docker Environment

A complete Docker environment for [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2): Fast, Direct LiDAR-Inertial-Visual Odometry with fully automated execution scripts.

## 🚀 Quick Start

Get FAST-LIVO2 running in 4 simple steps:

### 1. Initial Setup (One-time)
```bash
# Clone or download this repository
git clone git@github.com:j-albo/DECO2_project.git
cd DECO2_project/

# Run initial setup
chmod +x *.sh
./setup.sh
```

### 2. Add Your Data
Download rosbag files from [FAST-LIVO2-Dataset](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z).
```bash
# Place your .bag files in the bags/ directory
cp /path/to/your/dataset.bag bags/

# (Optional) Add configuration files
cp /path/to/calibration.yaml config/
```

### 3. Run FAST-LIVO2
Choose your preferred method:

**Interactive Selector (Recommended):**
```bash
./run_fast_livo2.sh
```

**Quick Run (First .bag found):**
```bash
./quick_run.sh
```

**Specific File:**
```bash
./run_fast_livo2.sh my_dataset.bag
```

### 4. Expected Result
- Real-time 3D mapping in rviz
- Camera feed visualization
- Trajectory tracking
- Automatic process management

## 📋 Overview

This Docker setup provides a ready-to-use environment for FAST-LIVO2 with:

- **Ubuntu 20.04 LTS** with ROS Noetic Desktop Full
- **All dependencies** pre-installed (PCL, OpenCV, Eigen, Sophus, etc.)
- **Pre-compiled FAST-LIVO2** workspace
- **GUI support** for rviz and visualization tools
- **Automated execution scripts** for one-command operation
- **Interactive file selection** and configuration management

## 🛠️ Prerequisites

### Required
- **Docker**: Version 20.10 or newer ([Installation Guide](https://docs.docker.com/get-docker/))
- **Hardware**: 
  - Minimum 8GB RAM
  - 15GB free disk space
  - GPU support recommended for better visualization

### Platform-Specific
**Linux (Recommended):**
- X11 server for GUI applications (usually pre-installed)

**Windows:**
- [Docker Desktop with WSL2](https://docs.docker.com/desktop/windows/install/)
- [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or similar X server for GUI support

**macOS:**
- [Docker Desktop](https://docs.docker.com/desktop/mac/install/)
- [XQuartz](https://www.xquartz.org/) for GUI support

## 📂 Project Structure

```
fast-livo2-docker/
├── docker/
│   ├── Dockerfile              # Docker image definition
│   ├── build_image.sh          # Image building script
│   └── run_container.sh        # Original container runner
├── run_fast_livo2.sh          # Main automated runner
├── quick_run.sh               # Simple quick execution
├── setup.sh                   # Initial setup script
├── usage_examples.sh          # Usage examples
├── bags/                      # Place .bag files here
├── config/                    # Configuration files
├── data/                      # Output data and results
└── README.md                  # This file
```

## 🎯 Usage Guide

### Automated Scripts

#### `run_fast_livo2.sh` - Main Automated Runner

Full-featured script with interactive selection and configuration options.

**Basic Usage:**
```bash
# Interactive file selector
./run_fast_livo2.sh

# Run with specific file
./run_fast_livo2.sh my_dataset.bag

# Run with custom configuration
./run_fast_livo2.sh -c calibration.yaml my_dataset.bag

# Build image before running
./run_fast_livo2.sh --build my_dataset.bag
```

**Advanced Options:**
```bash
# Use different launch file
./run_fast_livo2.sh -l mapping_mid360.launch dataset.bag

# Custom delay before rosbag playback
./run_fast_livo2.sh -d 15 dataset.bag

# Run without rviz
./run_fast_livo2.sh --no-rviz dataset.bag

# List available .bag files
./run_fast_livo2.sh --list-bags

# Show help
./run_fast_livo2.sh --help
```

#### `quick_run.sh` - Simple Quick Execution

Minimal script that automatically uses the first .bag file found.

```bash
./quick_run.sh
```

#### `setup.sh` - Initial Setup

One-time setup script that prepares the environment.

```bash
./setup.sh
```

### Manual Docker Usage

If you prefer manual control:

```bash
# Build image
./docker/build_image.sh

# Run container
./docker/run_container.sh

# Inside container - run FAST-LIVO2 manually
roslaunch fast_livo mapping_avia.launch

# In another terminal - play dataset
docker exec -it fast-livo2-container bash
rosbag play /home/developer/bags/your_dataset.bag

# In another terminal - open rviz
docker exec -it fast-livo2-container bash
rviz
```
