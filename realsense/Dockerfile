# Ubuntu release must match the ROS_DISTRO below:
#   humble / iron  -> jammy (22.04)
#   jazzy / kilted -> noble (24.04)
#   foxy           -> focal (20.04)
ARG UBUNTU_RELEASE=noble
FROM ubuntu:${UBUNTU_RELEASE} AS realsense_base

ARG ROS_DISTRO=jazzy

# Set automatically by BuildKit/buildx from --platform (amd64, arm64, ...). Re-declared
# inside the stage so the RUN lines below can branch on it; it is empty on a plain
# `docker build` with the legacy builder, which is treated as "native, whatever that is".
ARG TARGETARCH

# Cap the compiler parallelism of the two source builds below (librealsense2 via make,
# realsense-ros via colcon). Leaving it empty is a no-op -- librealsense2 gets the usual
# make -j$(nproc) and colcon its usual package-level parallelism, i.e. exactly the native
# behaviour this file has always had. Set it when cross-building for arm64 under QEMU on
# a many-core x86 host: each emulated compile job carries its own qemu-aarch64 process,
# so an unbounded $(nproc) (and colcon's nproc-wide package parallelism on top of it)
# reliably OOMs the builder part-way through librealsense2. Typical value: BUILD_JOBS=4.
ARG BUILD_JOBS=""

# Fail in seconds rather than 40 minutes in: packages.ros.org publishes ROS 2 debs for
# amd64 and arm64 only. A `--platform linux/arm/v7` (or riscv64) build gets all the way
# through the base apt layers and then dies inside `apt-get install ros-${ROS_DISTRO}-*`
# with a bare "Unable to locate package", which reads like a mirror outage rather than an
# unsupported architecture. TARGETARCH is empty under the legacy (non-buildx) builder, in
# which case the build is native and there is nothing to check.
RUN if [ -n "${TARGETARCH}" ] && [ "${TARGETARCH}" != "amd64" ] && [ "${TARGETARCH}" != "arm64" ]; then \
        echo "ERROR: unsupported target architecture '${TARGETARCH}' -- packages.ros.org ships ROS 2 ${ROS_DISTRO} debs for amd64 and arm64 only." >&2; \
        exit 1; \
    fi

# Setup environment
ENV HOME=/root
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
# Matches every other container in the stack (gui_orchestrator, arm_api2, etc.) --
# needed for realsense2_camera_node to launch cleanly against the source-built
# librealsense2/realsense2_camera pair below, see align_depth_realsense2_camera_fix.md.
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
    ros-${ROS_DISTRO}-generate-parameter-library \
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

# Modify .bashrc
RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Build argument to build realsense-ros (and librealsense2) from source instead of apt.
ARG BUILD_FROM_SOURCE="yes"
ENV REALSENSE_WS=${HOME}/realsense_ws

# The ros2 *stable* apt repo's librealsense2/realsense2-camera pairing (2.58.1/4.58.1,
# built 2026-05/06) has a silent align_depth bug: the Depth Module opens fine and raw
# depth publishes, but align_depth.enable:=true (and therefore
# aligned_depth_to_color/image_raw, which any RGB-D fusion consumer needs) never emits a
# single frame -- no error, no warning logged, it just never publishes.
#
# The ros2-testing repo's 2.58.3/4.58.3 pairing was *thought* fixed and pinned here
# briefly, but turned out to have the exact same silent-drop bug on live testing (see
# errors.md's "UPDATE 2026-08-25" and align_depth_realsense2_camera_fix.md) -- do not
# go back to installing ros2-testing's librealsense2/realsense2-camera for align_depth.
# The only version pair actually confirmed working end-to-end (color + aligned depth +
# HaMeR hand mesh, verified live on realsense_cont) is librealsense2 v2.58.2 +
# realsense2-camera 4.58.2, both source-built (2.58.2 is no longer resolvable via apt --
# ros2-testing only keeps its latest build, and that build has since moved past it) --
# hence BUILD_FROM_SOURCE defaults to "yes" here, not "no".
ARG REALSENSE_LIB_TAG="v2.58.2"
ARG REALSENSE_CAMERA_TAG="4.58.2"

# --- Option 1: install prebuilt packages from the ROS apt repo (NOT recommended --
# see the align_depth note above; kept only for a quick non-align_depth smoke test) ---
RUN if [ "$BUILD_FROM_SOURCE" = "no" ]; then \
        apt-get update && \
        apt-get install -y \
            "ros-${ROS_DISTRO}-librealsense2" \
            "ros-${ROS_DISTRO}-realsense2-camera" \
            "ros-${ROS_DISTRO}-realsense2-camera-msgs" \
            "ros-${ROS_DISTRO}-realsense2-description"; \
    fi

# --- Option 2 (default): build librealsense2 + realsense-ros from source, pinned to the
# exact tags confirmed working with align_depth. librealsense2 installs straight into the
# ROS prefix (not /usr/local) so realsense2_camera_node resolves it via the normal
# `source /opt/ros/${ROS_DISTRO}/setup.bash` -- no extra LD_LIBRARY_PATH/AMENT_PREFIX_PATH
# overlay needed at runtime (that overlay was the workaround needed for the original,
# not-baked-in ported build -- see align_depth_realsense2_camera_fix.md's "Still
# outstanding" section, which this Dockerfile change resolves). ---
RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        apt-get update && apt-get install -y \
            software-properties-common \
            libssl-dev \
            libusb-1.0-0-dev \
            libudev-dev \
            pkg-config \
            libgtk-3-dev \
            python3-dev \
            python3-pip; \
    fi

# udev rules only matter for the real device at *runtime* (this container is run
# --privileged with /dev bind-mounted, like every other driver container here) --
# setup_udev_rules.sh's own `sudo udevadm control --reload-rules && ... trigger` tail
# reliably fails in a Docker build sandbox (no running udev/systemd daemon), so it's
# `|| true`'d rather than let it silently abort the rest of this chain.
#
# FORCE_RSUSB_BACKEND=true is what makes this build arch-portable: it selects
# librealsense's userspace libusb backend instead of the V4L2 one, so the image needs no
# uvcvideo/hid kernel patches on the host. That matters most on arm64 (Jetson / RPi /
# Ampere hosts), where the patched-kernel path from librealsense's install docs is a
# non-starter -- and it costs nothing on amd64, which is why it was already set here.
RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        git clone --depth 1 --branch ${REALSENSE_LIB_TAG} https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
        cd /tmp/librealsense && \
        (./scripts/setup_udev_rules.sh || true) && \
        mkdir build && cd build && \
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true && \
        make -j"${BUILD_JOBS:-$(nproc)}" && make install && ldconfig && \
        rm -rf /tmp/librealsense; \
    fi

# rosdep init writes /etc/ros/rosdep/sources.list.d/20-default.list pointing every
# source at .../rosdistro/master/..., and GitHub's raw-content edge has been
# returning a hard "400: Bad Request" specifically for the master-branch-alias
# request to rosdep/base.yaml (other files under the same master ref, and base.yaml
# itself fetched by a pinned commit SHA, both resolve fine) -- so re-point that file
# at the current master commit SHA instead of the branch alias before updating.
RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        rosdep init || true && \
        ROSDISTRO_SHA=$(curl -sS https://api.github.com/repos/ros/rosdistro/commits/master | grep -m1 '"sha"' | sed -E 's/.*"sha": *"([a-f0-9]+)".*/\1/') && \
        [ -n "$ROSDISTRO_SHA" ] && \
        sed -i "s#/rosdistro/master/#/rosdistro/${ROSDISTRO_SHA}/#g" /etc/ros/rosdep/sources.list.d/20-default.list && \
        rosdep update; \
    fi

# --skip-keys librealsense2: realsense2_camera's package.xml <depend>s on
# librealsense2, which rosdep resolves to ros-${ROS_DISTRO}-librealsense2 --
# the broken apt package this Dockerfile source-builds around above. Without
# the skip, rosdep silently apt-installs it alongside the source build,
# putting two ABI-mismatched librealsense2.so.2 copies in the image and
# reintroducing the exact crash this file exists to avoid.
RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        mkdir -p ${REALSENSE_WS}/src && \
        git clone --depth 1 --branch ${REALSENSE_CAMERA_TAG} https://github.com/realsenseai/realsense-ros.git ${REALSENSE_WS}/src/realsense-ros && \
        bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                 cd ${REALSENSE_WS} && \
                 rosdep install --from-paths src --ignore-src -r -y --skip-keys librealsense2 && \
                 ${BUILD_JOBS:+MAKEFLAGS=-j}${BUILD_JOBS} \
                 colcon build --symlink-install ${BUILD_JOBS:+--parallel-workers 1} \
                   --cmake-args -DCMAKE_BUILD_TYPE=Release"; \
    fi

RUN if [ "$BUILD_FROM_SOURCE" = "yes" ]; then \
        echo "" >> ~/.bashrc && \
        echo "source ${REALSENSE_WS}/install/setup.bash" >> ~/.bashrc; \
    fi

WORKDIR ${REALSENSE_WS}

CMD ["bash"]
