# Ubuntu release must match the ROS_DISTRO below (see ../realsense/Dockerfile
# for the same convention):
#   humble / iron  -> jammy (22.04)
#   jazzy / kilted -> noble (24.04)
ARG UBUNTU_RELEASE=noble
FROM ubuntu:${UBUNTU_RELEASE} AS usb_cam_base

ARG ROS_DISTRO=jazzy

# Setup environment
ENV HOME=/root
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
# Matches every other container in the stack (gui_orchestrator, arm_api2,
# realsense, oak, etc.) -- required for DDS discovery across containers, see
# ../vendor_split/README.md's networking contract.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
    v4l-utils \
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
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-rqt-image-view

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

RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# CroboticSolutions' fork of usb_cam (branch apirsic/devel), same repo/branch
# already used in docker_files/ros2/jazzy/mediapipe/Dockerfile -- kept
# consistent with the rest of the org's tooling rather than pulling the
# ros-drivers/usb_cam upstream or the ros-${ROS_DISTRO}-usb-cam apt package.
ENV USB_CAM_WS=${HOME}/usb_cam_ws
RUN mkdir -p ${USB_CAM_WS}/src
WORKDIR ${USB_CAM_WS}/src

RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN --mount=type=ssh git clone -b apirsic/devel git@github.com:CroboticSolutions/usb_cam.git

# Default params file -- video_device/resolution/frame_id are the fields
# that most commonly need adjusting per-rig; see to_copy/params_1.yaml.
# Named params_1.yaml to match the existing reference in
# ../composers/demomotion/README.md ("edit params_1.yaml according to your
# camera settings").
COPY to_copy/params_1.yaml ${USB_CAM_WS}/src/usb_cam/config/params_1.yaml

WORKDIR ${USB_CAM_WS}
RUN rosdep init || true
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             rosdep update && \
             rosdep install --from-paths src --ignore-src -r -y"

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "" >> ~/.bashrc && \
    echo "source ${USB_CAM_WS}/install/setup.bash" >> ~/.bashrc

# Autostarts usb_cam_node_exe on container start (background, then exec's the
# passed CMD/command) -- see to_copy/ros_entrypoint.sh. Disable with
# `-e USB_CAM_AUTOSTART=0` if you need the container to come up idle.
COPY to_copy/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh

WORKDIR ${USB_CAM_WS}

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
