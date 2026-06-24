# syntax=docker/dockerfile:1.4
# Isaac Sim 4.2 with ROS 2 Jazzy
FROM nvcr.io/nvidia/isaac-sim:4.2.0 AS isaac_base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build arguments
ARG INSTALL_ROS2="yes"
ARG INSTALL_ROS2_BRIDGE="yes"
ARG PY_VENV=/isaac-sim/python.venv

# Environment setup
ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,video \
    TZ=Europe/Zagreb \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Set timezone
RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone

# Install additional system packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      apt-utils \
      bc \
      build-essential \
      cmake \
      curl \
      git \
      gnupg \
      htop \
      lsb-release \
      nano \
      net-tools \
      openssh-client \
      python3-pip \
      python3-venv \
      ranger \
      sudo \
      tmux \
      tmuxinator \
      wget && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 if requested
RUN if [ "${INSTALL_ROS2}" = "yes" ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
      apt-get update && \
      apt-get install -y --no-install-recommends \
        ros-${ROS2_DISTRO}-desktop-full \
        ros-${ROS2_DISTRO}-test-msgs \
        ros-${ROS2_DISTRO}-common-msgs \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-vcstool \
        ros-dev-tools && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Setup colcon mixins for ROS 2
RUN if [ "${INSTALL_ROS2}" = "yes" ]; then \
      colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
      colcon mixin update default; \
    fi

# Install ROS 2 - Isaac Sim bridge packages
RUN if [ "${INSTALL_ROS2_BRIDGE}" = "yes" ] && [ "${INSTALL_ROS2}" = "yes" ]; then \
      apt-get update && \
      apt-get install -y --no-install-recommends \
        ros-${ROS2_DISTRO}-tf2 \
        ros-${ROS2_DISTRO}-tf2-ros \
        ros-${ROS2_DISTRO}-geometry-msgs \
        ros-${ROS2_DISTRO}-sensor-msgs \
        python3-cv-bridge || true; \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Install Python development tools for Isaac Sim scripting
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir \
      numpy \
      scipy \
      matplotlib \
      jupyter \
      ipython \
      pyyaml \
      transforms3d \
      opencv-python

# Create symbolic link for convenience
RUN mkdir -p /root/workspace && \
    mkdir -p /root/isaac_projects

# Setup bashrc for environment activation
RUN echo "" >> /root/.bashrc && \
    echo "# Isaac Sim environment" >> /root/.bashrc && \
    echo "export NVIDIA_VISIBLE_DEVICES=all" >> /root/.bashrc && \
    echo "export NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,video" >> /root/.bashrc && \
    if [ "${INSTALL_ROS2}" = "yes" ]; then \
      echo "# ROS 2 setup" >> /root/.bashrc && \
      echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc; \
    fi

# Set working directory
WORKDIR /root/workspace

# Default command
CMD ["bash"]
