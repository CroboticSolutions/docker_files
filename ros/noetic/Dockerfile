# syntax=docker/dockerfile:1.3
FROM ubuntu:focal

# Build arguments
ARG BUILD_SLAM=true

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV HOME=/root
ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

RUN apt-get update

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    imagemagick \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \
    libncursesw5-dev \
    unzip

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update

# Install ROS
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full 

# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get install -y --no-install-recommends \
    python3-wstool \ 
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-rosdep \
    clang-format-10 \
    libeigen3-dev \
    libtf-conversions-dev

# Initialize rosdep
RUN rosdep init && rosdep update

# Init ROS workspace 
WORKDIR /root
RUN mkdir -p catkin_ws/src 
WORKDIR /root/catkin_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc 

# Install SLAM dependencies (conditional)
RUN bash -c 'if [ "$BUILD_SLAM" = "true" ]; then \
    apt-get install -y libpcl-dev libeigen3-dev; \
  fi'

# Build OpenCV 4.x (conditional for SLAM)
RUN bash -c 'if [ "$BUILD_SLAM" = "true" ]; then \
    cd /root && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && \ 
    git checkout 4.3.0 && \
    mkdir -p build && cd build && \
    cmake .. && \
    cmake --build . -j$(nproc) && \
    make install && \
    cd /root && rm -rf opencv; \
  fi'

# Build Sophus library (conditional for SLAM)
RUN bash -c 'if [ "$BUILD_SLAM" = "true" ]; then \
    mkdir -p /root/SLAM && cd /root/SLAM && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout a621ff && \
    sed -i -E \
      -e "s/unit_complex_\.real\(\) = ([0-9.]+);/unit_complex_.real(\1);/" \
      -e "s/unit_complex_\.imag\(\) = ([0-9.]+);/unit_complex_.imag(\1);/" \
      /root/SLAM/Sophus/sophus/so2.cpp && \
    mkdir build && cd build && \
    cmake .. && \
    make install -j$(nproc); \
  fi'

# Clone SLAM packages into catkin workspace (conditional)
WORKDIR /root/catkin_ws/src
RUN bash -c 'if [ "$BUILD_SLAM" = "true" ]; then \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    git clone https://github.com/xuankuzcr/rpg_vikit.git && \
    git clone https://github.com/hku-mars/FAST-LIVO2; \
  fi'

# Build workspace
WORKDIR /root/catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# ===================== ssh cloning ===================
ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
RUN --mount=type=ssh,id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts && \
    ssh-keyscan -H gitlab.com >> ~/.ssh/known_hosts

WORKDIR /root

CMD ["bash"]
