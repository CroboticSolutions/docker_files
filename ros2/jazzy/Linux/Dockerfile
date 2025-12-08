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
    x11-apps

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
    python3-vcstool

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

# Conditional LeRobot Python setup with Miniforge
RUN if [ "$LEROBOT_PY" = "yes" ]; then \
        apt-get install -y ffmpeg && \
        wget -q "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" -O /tmp/miniforge.sh && \
        bash /tmp/miniforge.sh -b -p $HOME/miniforge3 && \
        rm /tmp/miniforge.sh && \
        $HOME/miniforge3/bin/conda init bash && \
        echo "source $HOME/miniforge3/etc/profile.d/conda.sh" >> ~/.bashrc && \
        bash -c "source $HOME/miniforge3/etc/profile.d/conda.sh && conda create -y -n lerobot python=3.10" && \
        bash -c "source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate lerobot && pip install -e git+https://github.com/huggingface/lerobot.git#egg=lerobot"; \
    fi

RUN apt-get install -y python3-pip

# Add LeRobot conda initialization if LEROBOT_PY is enabled
RUN if [ "$LEROBOT_PY" = "yes" ]; then \
        echo "" >> ~/.bashrc && \
        echo "# LeRobot conda environment" >> ~/.bashrc && \
        echo "source $HOME/miniforge3/etc/profile.d/conda.sh" >> ~/.bashrc && \
        echo "conda activate lerobot" >> ~/.bashrc; \
    fi

# Conditional LeRobot ROS setup with SO_ARM workspace
RUN if [ "$LEROBOT_ROS" = "yes" ]; then \
        mkdir -p $HOME/so_arm_ws/src && \
        git clone https://github.com/JafarAbdi/ros2_so_arm100.git $HOME/so_arm_ws/src/ros2_so_arm100 && \
        git clone https://github.com/JafarAbdi/feetech_ros2_driver.git $HOME/so_arm_ws/src/feetech_ros2_driver && \
        git -C $HOME/so_arm_ws/src/ros2_so_arm100 submodule update --init --recursive && \
        rosdep init && \
        apt-get install -y python3-catkin-pkg && \ 
        bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash && cd $HOME/so_arm_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"; \
    fi

# Add SO_ARM workspace initialization if LEROBOT_ROS is enabled
RUN if [ "$LEROBOT_ROS" = "yes" ]; then \
        echo "" >> ~/.bashrc && \
        echo "# SO_ARM workspace" >> ~/.bashrc && \
        echo "source $HOME/so_arm_ws/install/setup.bash" >> ~/.bashrc; \
    fi

CMD ["bash"]

