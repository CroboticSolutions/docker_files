FROM ubuntu:noble AS ros2_base

# Setup environment
ENV HOME=/root

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS2_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
# All options for console format ="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
ARG PY_VENV=${HOME}/py_global

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    tmuxinator \
    wget \
    ranger \
    htop \
    libgl1 \
    libegl1 \
    libglu1-mesa \
    mesa-utils \
    x11-apps \
    python3-venv \
    python3-pip \
    python3-tk \
    librange-v3-dev \
    libserial-dev

# Set up Python virtual environment
RUN python3 -m venv ${PY_VENV} --system-site-packages
ENV PATH="${PY_VENV}/bin:$PATH"

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update

# Install ROS2
RUN apt-get update && apt-get install -y \
    ros-${ROS2_DISTRO}-desktop-full \
    ros-${ROS2_DISTRO}-test-msgs \
    ros-${ROS2_DISTRO}-generate-parameter-library

# Install ROS2 tools
RUN apt-get install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

# Colcon setup according to moveit tutorial
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default


RUN apt-get install -y \
    ros-${ROS2_DISTRO}-ackermann-msgs \
    ros-${ROS2_DISTRO}-backward-ros

# Build argument for Gazebo installation (default: no)
ARG INSTALL_GAZEBO="no"

# Conditional Gazebo Ionic installation
RUN if [ "$INSTALL_GAZEBO" = "yes" ]; then \
        curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
        apt-get update && \
        apt-get install -y gz-ionic; \
    fi

# Install ROS2-Gazebo bridge if Gazebo is installed
RUN if [ "$INSTALL_GAZEBO" = "yes" ]; then \
        apt-get install -y \
            ros-${ROS2_DISTRO}-ros-gz-sim \
            ros-${ROS2_DISTRO}-ros-gz-bridge || \
        apt-get install -y ros-${ROS2_DISTRO}-ros-gz || \
        echo "Warning: ROS-Gazebo bridge packages not available"; \
    fi

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc

# Install middleware
RUN apt-get install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Build arguments for LeRobot installation (default: no)
ARG LEROBOT_PY="no"
ARG LEROBOT_ROS="no"

RUN apt-get install -y python3-pip

# Conditional LeRobot Python setup with Miniforge
RUN if [ "$LEROBOT_PY" = "yes" ]; then \
        apt-get install -y ffmpeg && \
        pip3 install lerobot; \
    fi

# Conditional LeRobot ROS setup with SO_ARM workspace
RUN if [ "$LEROBOT_ROS" = "yes" ]; then \
        mkdir -p $HOME/arms_ws/src && \
        git clone https://github.com/JafarAbdi/ros2_so_arm100.git $HOME/arms_ws/src/ros2_so_arm100 && \
        git clone https://github.com/JafarAbdi/feetech_ros2_driver.git $HOME/arms_ws/src/feetech_ros2_driver && \
        git -C $HOME/arms_ws/src/ros2_so_arm100 submodule update --init --recursive; \
    fi

WORKDIR /root/arms_ws
# Add SO_ARM workspace initialization if LEROBOT_ROS is enabled
RUN if [ "$LEROBOT_ROS" = "yes" ]; then \
        echo "" >> ~/.bashrc && \
        echo "# SO_ARM workspace" >> ~/.bashrc && \
        echo "source $HOME/arms_ws/install/setup.bash" >> ~/.bashrc; \
    fi

# Clone arm_api2
WORKDIR /root/arms_ws/src
RUN git clone -b jazzy-compatibility https://github.com/CroboticSolutions/arm_api2.git
RUN git clone https://github.com/CroboticSolutions/arm_api2_msgs.git

ENV ROS_DISTRO=jazzy
# rosdep update
WORKDIR /root/arms_ws
RUN rosdep init
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Install necessary packages
RUN pip3 install \
    empy \
    catkin_pkg \
    lark 

# Install MoveIt Servo and MoveIt Visual tools
RUN apt-get update && apt-get install -y \ 
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    python3 -m colcon build --symlink-install" 

RUN echo "source ${PY_VENV}/bin/activate" >> $HOME/.bashrc

CMD ["bash"]

