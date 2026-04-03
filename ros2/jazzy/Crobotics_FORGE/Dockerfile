# syntax=docker/dockerfile:1.4
FROM ubuntu:noble AS ros2_base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# -----------------------------------------------------------------------------
# Build arguments
# -----------------------------------------------------------------------------
ARG USE_UR_ROBOT="no"
ARG INSTALL_GAZEBO="no"
ARG CROBOTIC_FORGE="no"
ARG PY_VENV=/root/py_global
ARG NODE_VERSION=20

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------
ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    TZ=Europe/Zagreb \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

ENV PATH="${PY_VENV}/bin:${PATH}"

ENV NVM_DIR=/root/.nvm

# -----------------------------------------------------------------------------
# Base system packages
# -----------------------------------------------------------------------------
RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      apt-utils \
      bc \
      build-essential \
      cmake \
      curl \
      git \
      gnupg \
      htop \
      libboost-dev \
      libegl1 \
      libgl1 \
      libglu1-mesa \
      librange-v3-dev \
      libserial-dev \
      lsb-release \
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
      tmuxinator \
      wget \
      x11-apps && \
    rm -rf /var/lib/apt/lists/*

# Python virtualenv available globally
RUN python3 -m venv "${PY_VENV}" --system-site-packages

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      nvidia-utils-580 \
      libnvidia-compute-580 && \
    rm -rf /var/lib/apt/lists/*

# Node.js via nvm (for npm-based tooling)
RUN curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \
    source "${NVM_DIR}/nvm.sh" && \
    nvm install "${NODE_VERSION}" && \
    nvm alias default "${NODE_VERSION}" && \
    nvm use default && \
    ln -sf "$(command -v node)" /usr/local/bin/node && \
    ln -sf "$(command -v npm)" /usr/local/bin/npm

ENV PATH="${NVM_DIR}/versions/node/v${NODE_VERSION}/bin:${PY_VENV}/bin:${PATH}"

# -----------------------------------------------------------------------------
# ROS 2 apt repository
# -----------------------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# -----------------------------------------------------------------------------
# ROS 2 core + common tooling
# -----------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-desktop-full \
      ros-${ROS2_DISTRO}-test-msgs \
      ros-${ROS2_DISTRO}-generate-parameter-library \
      ros-${ROS2_DISTRO}-ackermann-msgs \
      ros-${ROS2_DISTRO}-backward-ros \
      ros-${ROS2_DISTRO}-moveit \
      ros-${ROS2_DISTRO}-navigation2 \
      ros-${ROS2_DISTRO}-nav2-bringup \
      ros-${ROS2_DISTRO}-moveit-servo \
      ros-${ROS2_DISTRO}-moveit-visual-tools \
      ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
      python3-argcomplete \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-vcstool \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default

# -----------------------------------------------------------------------------
# Optional UR dependencies
# -----------------------------------------------------------------------------
RUN if [ "${USE_UR_ROBOT}" = "yes" ]; then \
      apt-get update && \
      apt-get install -y --no-install-recommends \
        ros-${ROS2_DISTRO}-ros2-control \
        ros-${ROS2_DISTRO}-ros2-controllers \
        socat && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# -----------------------------------------------------------------------------
# Optional Gazebo installation
# -----------------------------------------------------------------------------
RUN if [ "${INSTALL_GAZEBO}" = "yes" ]; then \
      curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
      apt-get update && \
      apt-get install -y --no-install-recommends gz-ionic && \
      apt-get install -y --no-install-recommends \
        ros-${ROS2_DISTRO}-gz-ros2-control \
        ros-${ROS2_DISTRO}-ros-gz-sim \
        ros-${ROS2_DISTRO}-ros-gz-bridge || \
      apt-get install -y --no-install-recommends ros-${ROS2_DISTRO}-ros-gz || true; \
      rm -rf /var/lib/apt/lists/*; \
    fi

# -----------------------------------------------------------------------------
# Workspace sources
# -----------------------------------------------------------------------------
WORKDIR /root/arms_ws/src

RUN git clone -b multiple_ur_robots https://github.com/CroboticSolutions/arm_api2.git && \
    git clone https://github.com/CroboticSolutions/arm_api2_msgs.git && \
    git clone https://github.com/ros-industrial/ur_msgs.git

RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -H github.com >> /root/.ssh/known_hosts

RUN --mount=type=ssh if [ "${CROBOTIC_FORGE}" = "yes" ]; then \
      git clone git@github.com:CroboticSolutions/aiortc_webrtc_ros.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/arm_api2_gui.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/ros2_dash_gui.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/ros2_robotiq_gripper.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/sam_ros2.git; \
    fi

RUN if [ "${CROBOTIC_FORGE}" = "yes" ]; then \
      source "${NVM_DIR}/nvm.sh" && \
      nvm use "${NODE_VERSION}" && \
      cd /root/arms_ws/src/arm_api2_gui && \
      npm install --legacy-peer-deps; \
    fi

RUN --mount=type=ssh if [ "${USE_UR_ROBOT}" = "yes" ]; then \
      git clone -b master https://github.com/UniversalRobots/Universal_Robots_Client_Library.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/Universal_Robots_ROS2_Driver.git && \
      git clone -b multiple_ur_robots git@github.com:CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation.git && \
      git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git; \
    fi

# -----------------------------------------------------------------------------
# Dependency resolution + build
# -----------------------------------------------------------------------------
WORKDIR /root/arms_ws

# Kreiranje direktorija i preuzimanje checkpointa samo za CROBOTIC_FORGE build
RUN if [ "${CROBOTIC_FORGE}" = "yes" ]; then \
      mkdir -p /root/arms_ws/src/segment-anything/checkpoints && \
      curl -L "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth" \
        -o /root/arms_ws/src/segment-anything/checkpoints/sam_vit_b_01ec64.pth; \
    fi

RUN rosdep init || true
RUN rosdep update && \
    # Skip non-Jazzy/unreleased keys and test/lint deps so container builds from mixed branches.
    rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS2_DISTRO}" \
      --dependency-types build \
      --dependency-types build_export \
      --dependency-types exec \
      --skip-keys "moveit_ros serial liburdfdom_tools gz_ros2_control ur_client_library ur_msgs ament_clang_format ament_clang_tidy joint_state_publisher_gui warehouse_ros_sqlite hardware_interface_testing ros2_controllers_test_nodes"

RUN pip3 install --no-cache-dir \
      empy \
      catkin_pkg \
      lark

RUN pip3 install --no-cache-dir \
      aiohttp \
      aiortc \
      flask \
      flask_cors \
      uvicorn \
      fastapi \
      websockets

RUN if [ "${CROBOTIC_FORGE}" = "yes" ]; then \
      pip3 install --no-cache-dir \
        git+https://github.com/facebookresearch/segment-anything.git \
        opencv-python \
        pycocotools \
        matplotlib \
        onnxruntime \
        onnx; \
    fi

RUN source /root/py_global/bin/activate && \
    python -m pip install --upgrade pip && \
    python -m pip uninstall -y torch torchvision torchaudio && \
    pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128

# Keep NumPy <2 as the final pip state for ROS cv_bridge compatibility.
RUN source /root/py_global/bin/activate && \
    pip uninstall -y numpy && \
    pip install --no-cache-dir --force-reinstall "numpy<2"

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install \
      --packages-skip robotiq_driver robotiq_controllers robotiq_hardware_tests

# -----------------------------------------------------------------------------
# Interactive shell defaults
# -----------------------------------------------------------------------------
RUN mkdir -p /root/.tmuxinator
COPY ur_gui_stack.yml /root/.tmuxinator/ur_gui_stack.yml

RUN echo "" >> /root/.bashrc && \
    echo "export NVM_DIR=${NVM_DIR}" >> /root/.bashrc && \
    echo "[ -s \"${NVM_DIR}/nvm.sh\" ] && source \"${NVM_DIR}/nvm.sh\"" >> /root/.bashrc && \
    echo "source ${PY_VENV}/bin/activate" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/arms_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]

