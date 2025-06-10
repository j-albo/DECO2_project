# FAST-LIVO2 Docker Environment - Real-time Version
# Professional setup for FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry
# Compatible with Ubuntu 20.04 and ROS Noetic
# Real-time support for Livox Mid-360 + ArduCam

FROM ubuntu:20.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Update base system
RUN apt-get update && apt-get upgrade -y

# Install system dependencies
RUN apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    apt-transport-https \
    udev \
    usbutils

# Configure ROS GPG key using modern method
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros.list > /dev/null

# Update package list after adding ROS repository
RUN apt-get update

# Install ROS Noetic Desktop Full
RUN apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-rqt* \
    ros-noetic-rviz

# Install development tools
RUN apt-get install -y \
    git \
    ssh \
    build-essential \
    cmake \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosinstall \
    iputils-ping \
    vim \
    nano

# Install FAST-LIVO2 specific dependencies
RUN apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev

# Install camera dependencies
RUN apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-geometry \
    ros-noetic-camera-calibration \
    ros-noetic-camera-info-manager \
    ros-noetic-usb-cam \
    v4l-utils \
    libv4l-dev

# Install additional real-time dependencies
RUN apt-get install -y \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher

# Clean apt cache
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (as root)
RUN rosdep init

# Install Sophus library globally (with compatibility patches)
RUN cd /tmp && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout a621ff && \
    # Apply compatibility patches for modern compilers \
    sed -i 's/unit_complex_.real() = 1./unit_complex_.real(1.)/g' sophus/so2.cpp && \
    sed -i 's/unit_complex_.imag() = 0./unit_complex_.imag(0.)/g' sophus/so2.cpp && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    # Create CMake configuration file manually \
    mkdir -p /usr/local/lib/cmake/Sophus && \
    echo 'set(Sophus_FOUND TRUE)' > /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    echo 'set(Sophus_INCLUDE_DIRS /usr/local/include)' >> /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    echo 'set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)' >> /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    echo 'set(SOPHUS_INCLUDE_DIRS /usr/local/include)' >> /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    echo 'set(SOPHUS_LIBRARIES /usr/local/lib/libSophus.so)' >> /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    cd / && rm -rf /tmp/Sophus

# Create non-root user
RUN useradd -ms /bin/bash developer && \
    echo 'developer ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    usermod -a -G dialout developer && \
    usermod -a -G video developer

# Switch to developer user
USER developer
WORKDIR /home/developer

# Configure ROS environment and update rosdep
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    rosdep update

# Create workspace directory
RUN mkdir -p /home/developer/dev_ws/src

# Set workspace as working directory
WORKDIR /home/developer/dev_ws/src

# Clone Livox SDK2 (for Mid-360)
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && \
    sudo make install

# Clone Livox ROS driver 2 (for Mid-360)
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# Clone rpg_vikit (FAST-LIVO2 specific fork)
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git

# Clone FAST-LIVO2 source code
RUN git clone https://github.com/hku-mars/FAST-LIVO2.git

# Return to workspace root
WORKDIR /home/developer/dev_ws

# Install ROS dependencies for workspace packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/developer/dev_ws && rosdep install --from-paths src --ignore-src -r -y || true"

# Build workspace using catkin_make
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH && catkin_make -DCMAKE_BUILD_TYPE=Release"

# Create real-time launch files
RUN mkdir -p /home/developer/dev_ws/src/FAST-LIVO2/launch/realtime

# Create environment setup script
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /home/developer/dev_ws/devel/setup.bash\n\
echo "FAST-LIVO2 Real-time environment ready!"\n\
echo "Usage examples:"\n\
echo "  roslaunch fast_livo realtime_mid360.launch"\n\
echo "  roslaunch fast_livo realtime_handheld.launch"\n\
echo "  rviz"\n\
exec "$@"' > /home/developer/entrypoint.sh && \
    chmod +x /home/developer/entrypoint.sh

# Create workspace setup script
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /home/developer/dev_ws/devel/setup.bash' > /home/developer/setup.bash && \
    chmod +x /home/developer/setup.bash

# Add setup to bashrc
RUN echo "source /home/developer/setup.bash" >> ~/.bashrc

# Set environment variables
ENV ROS_WORKSPACE=/home/developer/dev_ws

# Set entrypoint
ENTRYPOINT ["/home/developer/entrypoint.sh"]
CMD ["/bin/bash"]
