# DECO2 Project: FAST-LIVO2 Real-Time SLAM System

## 🎯 Project Overview

Complete real-time SLAM system using FAST-LIVO2 algorithm with Livox Mid-360 LiDAR. Evolution from offline bag processing to live hardware integration.

**Two Operation Modes:**
- 📁 **Bag Processing**: Process pre-recorded datasets (original stable version)
- 🚀 **Real-Time Hardware**: Live SLAM with Livox Mid-360 LiDAR (NEW!)

## 🚀 Quick Start

### Real-Time Hardware
```bash
cd docker/real-time
./build_fast_livo2.sh
./run_fast_livo2.sh

# Inside container:
test-livox
start-slam
```

### Bag Processing
```bash
cd docker/bag-processing
./run_fast_livo2.sh
```

## 🏗️ Hardware Requirements (Real-Time Mode)

### Essential Components
- **Livox Mid-360 LiDAR**
- **Power supply**: 12-24V, >15W
- **Ethernet cable** (Cat5e or better)
- **Host computer**: Ubuntu 20.04+ with Docker
- **USB Camera** (ArduCam recommended)

## 📖 Documentation

- 🔄 [Project Evolution](docs/project-evolution.md) - From bags to hardware
- 📁 [Original Bag Processing](docker/bag-processing/) - Stable offline processing

## 🛠️ Installation

### Prerequisites
```bash
# Install Docker
sudo apt update
sudo apt install docker.io docker-compose
sudo usermod -aG docker $USER
# Logout and login again
```

### Clone and Build
```bash
git clone https://github.com/j-albo/DECO2_project.git
cd DECO2_project/docker/real-time
./build_fast_livo2.sh
```

## 🎯 Usage

### 1. Hardware Setup
- Power on Livox Mid-360
- Connect via Ethernet cable
- Connect USB camera

### 2. Run System
```bash
./run_fast_livo2.sh

# Inside container, test hardware:
test-livox

# If successful, start SLAM:
start-slam
```

### 3. Expected Results
- **Network**: Automatic configuration to 192.168.1.5
- **Mid-360**: Detected at 192.168.1.170
- **Data Flow**: Point cloud and IMU data streaming
- **SLAM**: Real-time mapping in RViz

## 🔧 Troubleshooting

### Network Issues
```bash
# Check connectivity
ping 192.168.1.170

# Manual network config if needed
sudo ip addr add 192.168.1.5/24 dev <interface>
```

### Hardware Issues
- **Mid-360 LED not blue**: Check power supply
- **No network**: Check cable and interface
- **Permission errors**: Run with `sudo` if needed

## 📊 Project Evolution

This project demonstrates evolution from:
- **Academic**: Processing pre-recorded datasets
- **Practical**: Real-time SLAM with physical hardware

Both approaches use the same FAST-LIVO2 algorithm but with different data sources and complexity levels.

## 🏆 Results

- ✅ **Bag Processing**: Stable, reproducible results
- ✅ **Real-Time**: Live mapping and navigation capability
- ✅ **Docker**: One-command deployment for both modes
- ✅ **Documentation**: Complete setup guides

## 🤝 Contributing

1. For bag processing improvements: work in `docker/bag-processing/`
2. For real-time features: work in `docker/real-time/`
3. For documentation: update files in `docs/`

## 📝 License

MIT License - see LICENSE file for details.

## 🔗 References

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) - Original algorithm
- [Livox Mid-360](https://www.livoxtech.com/mid-360) - Hardware specifications
- [Docker](https://www.docker.com/) - Containerization platform

---

**Note**: The real-time hardware mode represents significant advancement from offline processing to practical SLAM applications ready for deployment.
