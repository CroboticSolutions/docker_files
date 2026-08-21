# Ubuntu release must match the ROS_DISTRO below:
#   humble / iron  -> jammy (22.04)
#   jazzy / kilted -> noble (24.04)
#   foxy           -> focal (20.04)
ARG UBUNTU_RELEASE=noble
FROM ubuntu:${UBUNTU_RELEASE} AS realsense_base

ARG ROS_DISTRO=jazzy

# Setup environment
ENV HOME=/root
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages
RUN apt-get update && apt-get install -q -y \
    apt-utils \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \
    ranger \
    htop \
    usbutils \
    udev \
    libgl1 \
    libglu1-mesa \
    mesa-utils

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update

# Install ROS2
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-generate-parameter-library

# Install ROS2 tools
RUN apt-get install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Modify .bashrc
RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Build argument to build realsense-ros (and librealsense2) from source instead of apt.
# Use "yes" when you need a newer librealsense2/firmware support than what's in the
# ROS apt repos, or want to track the ros2-master branch directly.
ARG BUILD_FROM_SOURCE="no"
ENV REALSENSE_WS=${HOME}/realsense_ws

# --- Option 1 (default): install prebuilt packages from the ROS apt repo ---
# https://github.com/realsenseai/realsense-ros - pulls in librealsense2 as a dependency.
RUN if [ "$BUILD_FROM_SOURCE" = "no" ]; then \
        apt-get update && \
        apt-get install -y "ros-${ROS_DISTRO}-realsense2-*"; \
    fi

# --- Option 2: build librealsense2 + realsense-ros from source ---
RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        apt-get update && apt-get install -y \
            software-properties-common \
            libssl-dev \
            libusb-1.0-0-dev \
            libudev-dev \
            pkg-config \
            libgtk-3-dev \
            python3-dev \
            python3-pip && \
        git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
        cd /tmp/librealsense && \
        ./scripts/setup_udev_rules.sh && \
        mkdir build && cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true && \
        make -j$(nproc) && make install && ldconfig && \
        rm -rf /tmp/librealsense && \
        mkdir -p ${REALSENSE_WS}/src && \
        git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master ${REALSENSE_WS}/src/realsense-ros && \
        rosdep init || true && rosdep update && \
        bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                 cd ${REALSENSE_WS} && \
                 rosdep install --from-paths src --ignore-src -r -y && \
                 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"; \
    fi

RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        echo "" >> ~/.bashrc && \
        echo "source ${REALSENSE_WS}/install/setup.bash" >> ~/.bashrc; \
    fi

WORKDIR ${REALSENSE_WS}

CMD ["bash"]
