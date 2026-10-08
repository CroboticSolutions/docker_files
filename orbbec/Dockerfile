# Orbbec Femto Bolt camera driver (OrbbecSDK_ROS2, CroboticSolutions fork,
# branch v2-main) for the PiPER + welding gun profiles. Same layout as
# ../usb_cam/Dockerfile: ROS 2 desktop, the driver cloned + colcon-built into
# its own workspace (ORBBEC_WS), launched by gui_orchestrator via docker exec.
#
# Build (the clone needs your GitHub SSH key forwarded into the build):
#   docker compose -f ../composers/vendor_drivers/docker-compose.yml build --ssh default orbbec
#
# Ubuntu release must match the ROS_DISTRO below:
#   humble / iron  -> jammy (22.04)
#   jazzy / kilted -> noble (24.04)
ARG UBUNTU_RELEASE=noble
FROM ubuntu:${UBUNTU_RELEASE} AS orbbec_base

ARG ROS_DISTRO=jazzy
ARG ORBBEC_BRANCH=v2-main
# Compile jobs for the colcon build. Unbounded parallelism froze a 14 GB laptop
# (2026-10-06) when run next to another heavy build; raise it on bigger machines.
ARG BUILD_JOBS=4

# Setup environment
ENV HOME=/root
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
# Matches every other container in the stack -- required for DDS discovery
# across containers, see ../vendor_split/README.md's networking contract.
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
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

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

# OrbbecSDK_ROS2 v2 build/runtime dependencies (its README's apt list; rosdep
# below picks up anything else declared in the package.xml files).
RUN apt-get install -y \
    libgflags-dev \
    libgoogle-glog-dev \
    nlohmann-json3-dev \
    libdw-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-backward-ros

RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

ENV ORBBEC_WS=${HOME}/orbbec_ws
RUN mkdir -p ${ORBBEC_WS}/src
WORKDIR ${ORBBEC_WS}/src

RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN --mount=type=ssh git clone -b ${ORBBEC_BRANCH} git@github.com:CroboticSolutions/OrbbecSDK_ROS2.git

# udev rules only matter on the *host* (this container runs --privileged with
# /dev bind-mounted, like every other driver container here); the script's
# udevadm reload fails in a build sandbox, hence `|| true`. Run
# orbbec_camera/scripts/install_udev_rules.sh once on the host as well.
RUN (cd ${ORBBEC_WS}/src/OrbbecSDK_ROS2/orbbec_camera/scripts && bash install_udev_rules.sh) || true

# Same rosdistro master-alias workaround as ../realsense/Dockerfile: GitHub's
# raw edge has been returning 400 for .../rosdistro/master/rosdep/base.yaml.
WORKDIR ${ORBBEC_WS}
RUN rosdep init || true && \
    ROSDISTRO_SHA=$(curl -sS https://api.github.com/repos/ros/rosdistro/commits/master | grep -m1 '"sha"' | sed -E 's/.*"sha": *"([a-f0-9]+)".*/\1/') && \
    [ -n "$ROSDISTRO_SHA" ] && \
    sed -i "s#/rosdistro/master/#/rosdistro/${ROSDISTRO_SHA}/#g" /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             rosdep install --from-paths src --ignore-src -r -y"

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             MAKEFLAGS=-j${BUILD_JOBS} colcon build --symlink-install --parallel-workers 1 \
               --event-handlers console_direct+ \
               --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "" >> ~/.bashrc && \
    echo "source ${ORBBEC_WS}/install/setup.bash" >> ~/.bashrc

WORKDIR ${ORBBEC_WS}

CMD ["bash"]
