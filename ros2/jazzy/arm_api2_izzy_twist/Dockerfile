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
RUN git clone -b mstigleitner/devel https://github.com/CroboticSolutions/arm_api2.git
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

# Use system Python (with python3-numpy headers) so rosidl_generator_py finds numpy/ndarrayobject.h
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && PATH=/usr/bin:\$PATH python3 -m colcon build --symlink-install" 

RUN echo "source ${PY_VENV}/bin/activate" >> $HOME/.bashrc



# Add ROS2 environment variables to .bashrc
RUN echo "source /root/arms_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export NVM_DIR=\"$HOME/.nvm\"" >> ~/.bashrc
RUN echo "[ -s \"$NVM_DIR/nvm.sh\" ] && \. \"$NVM_DIR/nvm.sh\"  # This loads nvm" >> ~/.bashrc
RUN echo "[ -s \"$NVM_DIR/bash_completion\" ] && \. \"$NVM_DIR/bash_completion\"  # This loads nvm bash_completion" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
RUN echo "export DISPLAY=:1" >> ~/.bashrc
RUN echo "xhost local:root" >> ~/.bashrc

# Allow git@github.com clones. Build with: docker build --ssh default -f Dockerfile .
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone piper_ros (branch matches robot_workspace tmux)
WORKDIR /root/arms_ws/src
RUN --mount=type=ssh git clone -b mstigla/devel git@github.com:CroboticSolutions/piper_ros.git

# Install MoveIt tools
RUN apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-pilz-industrial-motion-planner \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-hardware-interface

# Clone gazebo_models (used by piper_gazebo)
WORKDIR /root
RUN --mount=type=ssh git clone git@github.com:CroboticSolutions/gazebo_models.git

# Clone segment-anything
WORKDIR /root/arms_ws/src
RUN --mount=type=ssh git clone git@github.com:facebookresearch/segment-anything.git
WORKDIR /root/arms_ws/src/segment-anything
RUN . /root/py_global/bin/activate && pip install -e .

# Download sam_vit_b_01ec64.pth
WORKDIR /root/arms_ws/src/segment-anything/checkpoints
RUN wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth


WORKDIR /root
RUN . /root/py_global/bin/activate && pip install torch torchvision numpy opencv-python pillow

# --- Repos for robot_workspace tmux (branches match current workspace) ---
WORKDIR /root/arms_ws/src
# aiortc for WebRTC server_ros.py (ports 8081/8082/8083)
RUN --mount=type=ssh git clone -b main git@github.com:CroboticSolutions/aiortc_webrtc_ros.git aiortc
# Skip aiortc in colcon (pyproject.toml license format breaks colcon; we use pip install aiortc)
RUN touch /root/arms_ws/src/aiortc/COLCON_IGNORE
# SAM interactive (segment-anything ROS2)
RUN --mount=type=ssh git clone -b main git@github.com:CroboticSolutions/sam_ros2.git
# USB camera
RUN git clone -b main https://github.com/ros-drivers/usb_cam.git
# ArUco markers (for hand-eye and stream)
RUN --mount=type=ssh git clone -b mstigla/devel git@github.com:CroboticSolutions/aruco_markers.git
# Hand-eye calibration
RUN --mount=type=ssh git clone -b mstigla/devel git@github.com:CroboticSolutions/ros2_handeye_calibration.git
# React/Vite GUI (npm run dev)
RUN --mount=type=ssh git clone -b mstigleitner/devel git@github.com:CroboticSolutions/arm_api2_gui.git

# aiortc server_ros.py deps (rclpy/cv_bridge from ROS, rest from pip)
RUN . /root/py_global/bin/activate && pip install aiortc aiohttp av

# Pin NumPy <2 so ROS Jazzy cv_bridge (built for NumPy 1.x) loads; opencv may warn but usually works
RUN . /root/py_global/bin/activate && pip install "numpy<2"

# NVM + Node for arm_api2_gui (single RUN so nvm is available for npm)
ENV NVM_VERSION=0.40.1
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v${NVM_VERSION}/install.sh | bash \
    && . "$HOME/.nvm/nvm.sh" \
    && nvm install 20 \
    && nvm use 20 \
    && nvm alias default 20 \
    && cd /root/arms_ws/src/arm_api2_gui && npm ci \
    && ln -sf "$(which node)" /usr/local/bin/node \
    && ln -sf "$(which npm)" /usr/local/bin/npm \
    && ln -sf "$(which npx)" /usr/local/bin/npx

# Hand-eye calibration writes to this file at runtime
RUN mkdir -p /root/.ros

# Re-run rosdep for new packages, then build full workspace (piper_gazebo, piper_with_gripper_moveit, sam_ros2, usb_cam, aruco_markers, hand_eye_calibration, etc.)
# If hand_eye_calibration fails (e.g. missing detection_msgs), add that package to src and rosdep.
WORKDIR /root/arms_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y || true
# Use system Python so rosidl_generator_py finds numpy headers (arm_api2_msgs, aruco_markers_msgs, piper_msgs)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && PATH=/usr/bin:\$PATH colcon build --symlink-install"

# Tmuxinator config so container can run: tmuxinator robot_workspace
# (Copy from host: cp ~/.config/tmuxinator/robot_workspace.yml .  then docker build)
RUN mkdir -p /root/.config/tmuxinator
COPY to_copy/robot_workspace.yml /root/.config/tmuxinator/robot_workspace.yml

# Tmux config (copy from host: cp ~/.tmux.conf .  then docker build)
COPY to_copy/.tmux.conf /root/.tmux.conf

CMD ["bash"]


FROM ros2_base AS add_gui

WORKDIR ${HOME}

# Create a Python virtual environment for the GUI.
# GUI requires newer libraries (like numpy 2 which are not supported in ROS2 Jazzy)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv

ENV VIRTUAL_ENV=${HOME}/gui_venv
RUN python3 -m venv ${VIRTUAL_ENV}

# Install Python dependencies globally
RUN pip install --upgrade pip
RUN pip install \
    websockets

# Install Python dependencies in the virtual environment
RUN . ${VIRTUAL_ENV}/bin/activate && pip install --upgrade pip
RUN . ${VIRTUAL_ENV}/bin/activate && pip install \
    pyyaml \
    dash \
    dash-bootstrap-components \
    dash-extensions \
    dash[diskcache] \
    opencv-python \
    websockets \
    gdown

# Install Python API for the chosen LLM backend
ARG LLM_BACKEND="google"
RUN if [ "$LLM_BACKEND" = "ollama" ]; then . ${VIRTUAL_ENV}/bin/activate && pip install ollama; fi
RUN if [ "$LLM_BACKEND" = "google" ]; then . ${VIRTUAL_ENV}/bin/activate && pip install google-genai; fi

# We need this to use private ssh repos in Dockerfile: https://stackoverflow.com/a/53548076
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone the GUI
WORKDIR /root/arms_ws/src
RUN --mount=type=ssh git clone --single-branch --branch jazzy git@github.com:CroboticSolutions/ros2_dash_gui.git

# Add a demo video bag
WORKDIR /root/arms_ws/src/ros2_dash_gui
RUN . ${VIRTUAL_ENV}/bin/activate && bash download_demo_bag.sh

# Build the GUI as part of arms_ws (unset VIRTUAL_ENV; use system Python so colcon/rosidl find numpy headers)
WORKDIR /root/arms_ws/
RUN bash -c "unset VIRTUAL_ENV; source /opt/ros/jazzy/setup.bash; PATH=/usr/bin:\$PATH colcon build --symlink-install"

CMD ["bash"]

