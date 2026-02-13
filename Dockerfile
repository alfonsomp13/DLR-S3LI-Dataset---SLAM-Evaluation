# Dockerfile for DLR S3LI Dataset SLAM Evaluation
FROM ubuntu:20.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    python3 \
    python3-pip \
    python3-dev \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libsuitesparse-dev \
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libatlas-base-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    libepoxy-dev \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    xvfb \
    gdb \
    vim \
    htop \
    tmux \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Noetic (required for some SLAM systems)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install numpy scipy matplotlib pandas \
    opencv-python opencv-contrib-python \
    pyyaml evo pyquaternion tqdm

# Set up workspace
WORKDIR /workspace

# Build Pangolin (required by ORB-SLAM3)
ARG PANGOLIN_COMMIT=v0.6
RUN git clone https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin && \
    cd /tmp/Pangolin && \
    git checkout ${PANGOLIN_COMMIT} && \
    sed -i 's/-Werror=maybe-uninitialized//g' CMakeLists.txt && \
    sed -i 's/-Werror=vla//g' CMakeLists.txt && \
    sed -i 's/=maybe-uninitialized//g' CMakeLists.txt && \
    sed -i 's/=vla//g' CMakeLists.txt && \
    sed -i 's/-Werror//g' CMakeLists.txt && \
    sed -i 's/-Wno-null-pointer-arithmetic//g' CMakeLists.txt && \
    sed -i 's/-Wno-null-pointer-subtraction//g' CMakeLists.txt && \
    if [ -f components/pango_image/CMakeLists.txt ]; then sed -i 's/-Wno-deprecated-register//g' components/pango_image/CMakeLists.txt; fi && \
    rm -rf build && mkdir build && cd build && \
    cmake .. -DBUILD_PANGO_IMAGE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF && \
    make -j4 && make install && ldconfig && \
    rm -rf /tmp/Pangolin

# Build ORB-SLAM3
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git && \
    cd ORB_SLAM3 && \
    chmod +x build.sh && \
    sed -i 's/march=native/march=x86-64/g' CMakeLists.txt && \
    sed -i 's/OpenCV 4.4/OpenCV 4.2/' CMakeLists.txt && \
    sed -i '/add_definitions(-DCOMPILEDWITHC11)/a add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY -DEIGEN_DONT_VECTORIZE)' CMakeLists.txt && \
    ./build.sh || true

# Build VINS-Fusion
RUN mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git && \
    cd .. && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -j4 || true

# Build OpenVINS
RUN cd catkin_ws/src && \
    git clone https://github.com/rpng/open_vins.git && \
    cd .. && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -j4 || true

# Build Basalt (VIO system)
RUN apt-get update && apt-get install -y \
    libtbb-dev \
    libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive https://gitlab.com/VladyslavUsenko/basalt.git && \
    cd basalt && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j4 || true

# Create directories for dataset and results
RUN mkdir -p /workspace/dataset \
    /workspace/results \
    /workspace/scripts \
    /workspace/configs

# Copy evaluation scripts
COPY evaluation_scripts/ /workspace/scripts/
COPY configs/ /workspace/configs/

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export ORB_SLAM3_ROOT=/workspace/ORB_SLAM3" >> ~/.bashrc

# Keep container running
CMD ["tail", "-f", "/dev/null"]
