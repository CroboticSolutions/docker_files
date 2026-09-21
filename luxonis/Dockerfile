FROM ubuntu:jammy AS ros2_base

# Builds depthai-ros (humble branch, v2.12.2) for an OAK-D SR camera.
# See README.md in this folder for build/run instructions and topic reference.
#
# Builds for linux/amd64 and linux/arm64. Everything apt-installed below (including
# ros-humble-desktop-full and ros-humble-ffmpeg-image-transport-msgs) is published for
# both arches on packages.ros.org/ros2/ubuntu jammy, and depthai-core is source-built
# here, so nothing needs substituting per-arch -- but see BUILD_JOBS/ROS_METAPACKAGE
# below before cross-building under QEMU, and README.md's "Cross-building for arm64".

# Set automatically by BuildKit/buildx from --platform (amd64, arm64, ...). Empty under
# the legacy (non-buildx) builder, in which case the build is native.
ARG TARGETARCH

# depthai-core is a large C++ source build and is the memory bottleneck of this image,
# which is why the colcon step below is pinned to one package at a time with a capped
# MAKEFLAGS rather than the usual $(nproc). Under QEMU every compiler job also carries a
# qemu-aarch64 process, so leave this at 2 (or drop it to 1) for an arm64 cross-build;
# raise it only for a native build on a machine with RAM to spare.
ARG BUILD_JOBS=2

# desktop-full pulls the full Gazebo/perception stack, which this driver container does
# not use and which roughly triples both the download and the emulated install time.
# Kept as the default so the amd64 image is unchanged; the arm64 branch can pass
# --build-arg ROS_METAPACKAGE=ros-humble-ros-base (or ros-humble-desktop if RViz is
# wanted in-container) instead.
ARG ROS_METAPACKAGE=ros-humble-desktop-full

# Fail in seconds rather than deep into the build: packages.ros.org publishes ROS 2 debs
# for amd64 and arm64 only. A linux/arm/v7 or riscv64 build otherwise gets through the
# base apt layers and then dies inside `apt-get install ros-humble-*` with a bare
# "Unable to locate package", which reads like a mirror outage, not a wrong --platform.
RUN if [ -n "${TARGETARCH}" ] && [ "${TARGETARCH}" != "amd64" ] && [ "${TARGETARCH}" != "arm64" ]; then \
        echo "ERROR: unsupported target architecture '${TARGETARCH}' -- packages.ros.org ships ROS 2 humble debs for amd64 and arm64 only." >&2; \
        exit 1; \
    fi

# Setup environment
ENV HOME=/root

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS2_DISTRO=humble
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
    htop \
    libusb-1.0-0-dev \
    python3-pip

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update

# Install ROS2
RUN apt-get install -y ${ROS_METAPACKAGE}

# Install ROS2 tools
RUN apt-get install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep

# Colcon setup
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Install middleware. Must match the other host-networked ROS2 containers on this
# machine (e.g. composers/demomotion) so DDS discovery finds this container's topics.
RUN apt-get install -y ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# ffmpeg_image_transport_msgs is a depthai_ros_driver dependency; it's occasionally
# missing from the apt mirror, in which case fall back to building it from source.
RUN apt-get update && \
    (apt-get install -y ros-${ROS2_DISTRO}-ffmpeg-image-transport-msgs || true)

RUN rosdep init || true
RUN rosdep update

ENV DEPTHAI_WS=/root/depthai_ws
RUN mkdir -p ${DEPTHAI_WS}/src
WORKDIR ${DEPTHAI_WS}/src

RUN git clone --branch humble https://github.com/luxonis/depthai-ros.git depthai-ros

# Fallback if the apt package above was unavailable
RUN dpkg -s ros-${ROS2_DISTRO}-ffmpeg-image-transport-msgs > /dev/null 2>&1 || \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs.git ffmpeg_image_transport_msgs

# depthai_examples breaks the build (conflicting deps) and isn't needed to run the
# driver, so drop the dependency on it before resolving/building the workspace.
RUN sed -i '/<depend>depthai_examples<\/depend>/d' ${DEPTHAI_WS}/src/depthai-ros/depthai_ros_driver/package.xml && \
    sed -i '/<exec_depend>depthai_examples<\/exec_depend>/d' ${DEPTHAI_WS}/src/depthai-ros/depthai-ros/package.xml

# OAK-D SR has no RGB sensor: force the Depth pipeline and publish the raw +
# rectified right mono image so it lines up with /oak/stereo/image_raw (both are
# aligned to CAM_C / the right camera on this variant).
COPY to_copy/oak_d_sr.yaml ${DEPTHAI_WS}/src/depthai-ros/depthai_ros_driver/config/oak_d_sr.yaml

WORKDIR ${DEPTHAI_WS}
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys depthai_examples

# colcon's cmake step needs empy's old (3.x) API under Humble; empy 4.x breaks it.
RUN pip3 install catkin_pkg empy==3.3.4 lark

RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    MAKEFLAGS="-j${BUILD_JOBS}" colcon build --symlink-install \
      --parallel-workers 1 \
      --packages-skip depthai_examples \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${DEPTHAI_WS}/install/setup.bash" >> ~/.bashrc

WORKDIR ${DEPTHAI_WS}
CMD ["bash"]
