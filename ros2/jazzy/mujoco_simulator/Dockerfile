# syntax=docker/dockerfile:1.4
FROM ubuntu:noble AS ros2_mujoco

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build arguments
ARG INSTALL_ROS2="yes"
ARG PY_VENV=/root/py_global
ARG MUJOCO_VERSION=3.1.0

# Environment setup
ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    TZ=Europe/Zagreb \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

ENV PATH="${PY_VENV}/bin:${PATH}"

# Set timezone
RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone

# Base system packages
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
      libegl1 \
      libgl1 \
      libglu1-mesa \
      libxml2-dev \
      libxslt1-dev \
      libhdf5-dev \
      libopengl0 \
      mesa-utils \
      nano \
      net-tools \
      openssh-client \
      python3-pip \
      python3-tk \
      python3-venv \
      ranger \
      sudo \
      tmux \
      wget \
      x11-apps && \
    rm -rf /var/lib/apt/lists/*

# Setup Python virtual environment
RUN python3 -m venv "${PY_VENV}" --system-site-packages

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
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-vcstool \
        ros-dev-tools && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Setup colcon mixins
RUN if [ "${INSTALL_ROS2}" = "yes" ]; then \
      colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
      colcon mixin update default; \
    fi

# Install MuJoCo dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libgl1-mesa-glx \
      libglx0 \
      libxext6 \
      libxrender-dev && \
    rm -rf /var/lib/apt/lists/*

# Install MuJoCo via pip in virtual environment
RUN source "${PY_VENV}/bin/activate" && \
    pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir \
      mujoco==${MUJOCO_VERSION} \
      dm_control \
      numpy \
      scipy \
      matplotlib \
      jupyter \
      ipython

# Install additional visualization and robotics tools
RUN source "${PY_VENV}/bin/activate" && \
    pip install --no-cache-dir \
      pyyaml \
      transforms3d

# Setup bashrc for automated environment loading
RUN echo "" >> /root/.bashrc && \
    echo "# Python virtual environment activation" >> /root/.bashrc && \
    echo "source ${PY_VENV}/bin/activate" >> /root/.bashrc && \
    if [ "${INSTALL_ROS2}" = "yes" ]; then \
      echo "# ROS 2 setup" >> /root/.bashrc && \
      echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc; \
    fi

# Set working directory
WORKDIR /root/workspace

# Default command
CMD ["bash"]
