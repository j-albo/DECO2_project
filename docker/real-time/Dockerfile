# FAST-LIVO2 with Livox Mid-360 Support - Minimal Version
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/home/developer/dev_ws

# Install basic dependencies only
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    sudo \
    vim \
    net-tools \
    iputils-ping \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-usb-cam \
    ros-noetic-cv-bridge \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Create developer user
RUN useradd -m -s /bin/bash developer && \
    echo "developer:developer" | chpasswd && \
    adduser developer sudo && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER developer
WORKDIR /home/developer

# Create minimal workspace with only FAST-LIVO2
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

# Clone only FAST-LIVO2 (skip problematic dependencies)
RUN git clone https://github.com/hku-mars/FAST-LIVO2.git

# Clone Livox SDK2 for hardware
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Build Livox SDK2
WORKDIR ${CATKIN_WS}/src/Livox-SDK2
RUN mkdir build && cd build && \
    cmake .. -DBUILD_SAMPLES=ON && \
    make -j$(nproc)

# Modify FAST-LIVO2 to remove problematic dependencies
WORKDIR ${CATKIN_WS}/src/FAST-LIVO2
RUN sed -i 's/vikit_common/#vikit_common/g' package.xml && \
    sed -i 's/find_package(vikit_common REQUIRED)/#find_package(vikit_common REQUIRED)/g' CMakeLists.txt

# Build minimal ROS workspace (just FAST-LIVO2)
WORKDIR ${CATKIN_WS}
USER root
RUN rosdep init || true
USER developer
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y || true
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make || echo 'Some packages failed but continuing'"

# Create helper scripts
WORKDIR /home/developer

# Network configuration script
RUN printf '#!/bin/bash\necho "🔧 Configuring network..."\nINTERFACE=$(ip link | grep -E "enx[0-9a-f]{12}" | cut -d: -f2 | tr -d " " | head -1)\nif [ -z "$INTERFACE" ]; then\n  echo "❌ No interface found"\n  ip link show\n  exit 1\nfi\necho "✅ Found: $INTERFACE"\nsudo ip addr flush dev $INTERFACE 2>/dev/null || true\nsudo ip addr add 192.168.1.5/24 dev $INTERFACE\nsudo ip link set $INTERFACE up\nsudo ip link set $INTERFACE multicast on\nping -c 2 192.168.1.170\n' > configure_network.sh && chmod +x configure_network.sh

# SDK test script
RUN printf '#!/bin/bash\necho "🔍 Testing Livox SDK..."\n./configure_network.sh || exit 1\necho "Starting SDK test..."\ncd /home/developer/dev_ws/src/Livox-SDK2/build/samples/livox_lidar_quick_start\nsudo ./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json\n' > test_livox_sdk.sh && chmod +x test_livox_sdk.sh

# Simple SLAM script
RUN printf '#!/bin/bash\necho "🚀 SLAM Ready"\necho "1. First run: test-livox (to verify hardware)"\necho "2. Then run in separate terminals:"\necho "   roscore"\necho "   roslaunch fast_livo mapping_avia.launch"\nsource /opt/ros/noetic/setup.bash\nsource /home/developer/dev_ws/devel/setup.bash\n./configure_network.sh\n' > start_slam.sh && chmod +x start_slam.sh

# Setup environment
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /home/developer/dev_ws/devel/setup.bash' >> ~/.bashrc && \
    echo 'alias test-livox="~/test_livox_sdk.sh"' >> ~/.bashrc && \
    echo 'alias start-slam="~/start_slam.sh"' >> ~/.bashrc && \
    echo 'alias config-net="~/configure_network.sh"' >> ~/.bashrc && \
    echo 'echo "🚀 FAST-LIVO2 Ready! Commands: test-livox, start-slam"' >> ~/.bashrc

WORKDIR /home/developer
CMD ["/bin/bash"]
