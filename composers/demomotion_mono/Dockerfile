# syntax=docker/dockerfile:1.7
#
# DemoMotion u jednom kontejneru: GUI (gateway + runtime + debug konzola),
# dash bridge, WebRTC, a runtime_service sam pokreće Piper driver, MoveIt,
# arm_api2, Orbbec, SAM2, WiLoR/hand_teleop, MegaPose/M3T i FoundationPose
# kao child procese.
#
# Putanje su IDENTIČNE razvojnom kontejneru (/root/arms_ws, /root/py_global,
# /root/arm_perception, ~/.ros/calibration ...), pa kod ne treba mijenjati.
# Modeli nisu u imageu: compose montira $DEMOMOTION_MODELS na /models, a image
# ima symlinkove s putanja koje kod očekuje (vidi "modeli" niže).
#
# Build ide kroz ./demomotion.sh build (SSH agent za privatne repoe, SRC_REVISIONS).
# Prva verzija: finalni image = build stage (veći, ali bez iznenađenja s
# nedostajućim runtime ovisnostima). Smanjivanje kasnije.

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------- base
# Ubuntu 24.04 + ROS Jazzy desktop-full (RViz, Gazebo; isto kao razvojni kontejner).
# Nakon prvog uspješnog builda pinati digest:
#   docker buildx imagetools inspect osrf/ros:jazzy-desktop-full
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base
ARG ROS_DISTRO
ARG NODE_MAJOR=24
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Europe/Zagreb \
    TERM=xterm-256color \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///root/arms_ws/cyclonedds_localhost.xml \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# apt cache mount radi samo bez docker-clean hooka
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Popis iz `apt-mark showmanual` razvojnog kontejnera (bez desktop alata).
# Ostatak ROS ovisnosti dovuče rosdep u build stageu.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config git curl wget ca-certificates gnupg openssh-client sudo \
      python3-dev python3-pip python3-venv python3-vcstool \
      python3-colcon-common-extensions python3-colcon-mixin python3-pynput python3-tk \
      can-utils ethtool iproute2 net-tools usbutils udev lsof psmisc procps socat \
      tmux tmuxinator ffmpeg gdb htop nano \
      libeigen3-dev libpcl-dev libopencv-dev libserial-dev libboost-dev libtbb-dev \
      librange-v3-dev libatlas-base-dev libopenblas-dev libhdf5-dev \
      libglew-dev libglfw3-dev libomp-dev libgl1-mesa-dev libegl1 libglu1-mesa mesa-utils \
      libgflags-dev nlohmann-json3-dev libdw-dev libusb-1.0-0-dev \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-servo ros-${ROS_DISTRO}-moveit-visual-tools \
      ros-${ROS_DISTRO}-moveit-planners-chomp ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
      ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-gz-ros2-control ros-${ROS_DISTRO}-parallel-gripper-controller \
      ros-${ROS_DISTRO}-tf-transformations ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-librealsense2 ros-${ROS_DISTRO}-depthai-v3 \
      ros-${ROS_DISTRO}-robot-calibration ros-${ROS_DISTRO}-warehouse-ros-sqlite \
      ros-${ROS_DISTRO}-generate-parameter-library ros-${ROS_DISTRO}-backward-ros

# Node za build frontenda (vite)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    curl -fsSL https://deb.nodesource.com/setup_${NODE_MAJOR}.x | bash - && \
    apt-get install -y --no-install-recommends nodejs

# Apt python3-matplotlib (ovisnost rqt-plota) ima nspkg.pth koji pri startu
# učita stari mpl_toolkits preko pip matplotliba (brain: runtime_environment).
RUN for f in /usr/lib/python3/dist-packages/matplotlib-*-nspkg.pth; do \
      [ -e "$f" ] && dpkg-divert --local --rename --divert "$f.disabled" --add "$f"; \
    done; true

WORKDIR /root/arms_ws
COPY config/cyclonedds_localhost.xml config/setup_ros_domain.sh /root/arms_ws/

# ---------------------------------------------------------------- python
# py_global: venv sa --system-site-packages (rclpy i cv_bridge iz apta ostaju vidljivi).
FROM base AS python
RUN python3 -m venv --system-site-packages /root/py_global
ENV VIRTUAL_ENV=/root/py_global \
    PATH=/root/py_global/bin:$PATH
COPY requirements/ /opt/demomotion/requirements/
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install -r /opt/demomotion/requirements/torch.txt \
      --index-url https://download.pytorch.org/whl/cu128 \
      --extra-index-url https://pypi.org/simple
# --no-deps: točno skup iz razvojnog kontejnera, bez resolvera
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --no-deps -r /opt/demomotion/requirements/py_global.lock && \
    pip install --no-deps -r /opt/demomotion/requirements/py_global.git.txt && \
    touch /root/py_global/lib/python3.12/site-packages/mpl_toolkits/__init__.py && \
    (pip check || echo "pip check: nepodudarnosti postoje i u razvojnom kontejneru, samo upozorenje")

# ---------------------------------------------------------------- fpdeps
# FoundationPose: CUDA 12.8 toolkit (/usr/local/cuda-12.8, CUDA_HOME koji kod
# očekuje) i CUDA ekstenzije u /root/foundationpose/venv. Ovdje, a ne nakon
# kloniranja koda: pytorch3d se kompajlira dugo i ne ovisi o našim granama.
# Build nema GPU, pa se arhitekture navode ručno:
#   8.6 = RTX 30xx, 8.9 = RTX 40xx, 12.0 = RTX 50xx (Blackwell).
FROM python AS fpdeps
ARG FP_CUDA_ARCHS="8.6;8.9;12.0"
# nvcc procesi imaju po 2-3 GB RSS (build_fp.sh: 8 jobova je zamrznulo laptop s 14 GB)
ARG FP_MAX_JOBS=2
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    curl -fsSLo /tmp/cuda-keyring.deb \
      https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
      ninja-build libboost-system-dev libboost-program-options-dev \
      cuda-nvcc-12-8 cuda-cudart-dev-12-8 cuda-cccl-12-8 cuda-nvrtc-dev-12-8 \
      cuda-nvtx-12-8 cuda-profiler-api-12-8 \
      libcublas-dev-12-8 libcusparse-dev-12-8 libcusolver-dev-12-8 libcurand-dev-12-8
ENV CUDA_HOME=/usr/local/cuda-12.8
# venv kao lokalno: vidi py_global (torch) preko .pth, a svoj numpy 2 drži lokalno
RUN python3 -m venv --system-site-packages /root/foundationpose/venv && \
    echo /root/py_global/lib/python3.12/site-packages \
      > /root/foundationpose/venv/lib/python3.12/site-packages/py_global_readonly.pth
RUN --mount=type=cache,target=/root/.cache/pip \
    /root/foundationpose/venv/bin/pip install --no-deps -r /opt/demomotion/requirements/foundationpose.lock
RUN --mount=type=cache,target=/root/.cache/pip \
    export PATH=$CUDA_HOME/bin:$PATH TORCH_CUDA_ARCH_LIST="${FP_CUDA_ARCHS}" FORCE_CUDA=1 \
           MAX_JOBS=${FP_MAX_JOBS} MAKEFLAGS=-j${FP_MAX_JOBS} && \
    /root/foundationpose/venv/bin/pip install --no-deps --no-build-isolation \
      -r /opt/demomotion/requirements/foundationpose.git.txt

# ---------------------------------------------------------------- sources
FROM fpdeps AS sources
ARG SAM2_COMMIT=2b90b9f5ceec907a1c18123530e92e794ad901a4
COPY repos.yaml /root/arms_ws/repos.yaml
# SRC_REVISIONS = hash vrhova svih grana (računa ga demomotion.sh). Kad se neka
# grana pomakne, mijenja se i on, pa se sve od ove točke ponovno builda; apt i
# pip slojevi iznad ostaju u cacheu.
ARG SRC_REVISIONS=unknown
RUN --mount=type=ssh \
    echo "SRC_REVISIONS=${SRC_REVISIONS}" && \
    mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -t ed25519,rsa,ecdsa github.com >> /root/.ssh/known_hosts 2>/dev/null && \
    vcs import --shallow --input repos.yaml src && \
    git init -q src/third_party/sam2 && \
    git -C src/third_party/sam2 remote add origin https://github.com/facebookresearch/sam2.git && \
    git -C src/third_party/sam2 fetch -q --depth 1 origin ${SAM2_COMMIT} && \
    git -C src/third_party/sam2 checkout -q FETCH_HEAD && \
    vcs export --exact src > /root/arms_ws/VERSIONS.yaml
# COLCON_IGNORE koji u razvojnom kontejneru postoje samo lokalno (nisu u gitu)
RUN touch src/third_party/COLCON_IGNORE src/misc/webrtc_video_server/COLCON_IGNORE

# ---------------------------------------------------------------- build
FROM sources AS build
ARG ROS_DISTRO
ARG COLCON_WORKERS=4
ARG ROSDEP_SKIP_KEYS=""
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && rosdep update --rosdistro ${ROS_DISTRO} && \
    # bez venva na PATH-u: rosdep pip ključevi ne smiju dirati zaključani py_global
    (PATH=/usr/sbin:/usr/bin:/sbin:/bin rosdep install --from-paths src --ignore-src -y -r \
       --rosdistro ${ROS_DISTRO} --skip-keys "${ROSDEP_SKIP_KEYS}" || \
     echo "rosdep: dio ovisnosti nije instaliran (vidi gore); ako colcon padne, dodaj ih u apt listu")

# Bez --symlink-install (kod je u imageu, ne mijenja se). Orbbec traži Release.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --base-paths src --parallel-workers ${COLCON_WORKERS} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --event-handlers console_cohesion+ && \
    rm -rf log

# SAM2 kao biblioteka u py_global (editable, kao u razvojnom kontejneru)
RUN --mount=type=cache,target=/root/.cache/pip \
    SAM2_BUILD_CUDA=0 pip install --no-deps --no-build-isolation -e src/third_party/sam2

# MegaPose i pym3t (M3T): zasebni venvovi koji preko .pth vide py_global, isto kao lokalno
RUN --mount=type=cache,target=/root/.cache/pip \
    for v in src/megapose6d/venv src/third_party/pym3t/.venv; do \
      python3 -m venv --system-site-packages "$v" && \
      echo /root/py_global/lib/python3.12/site-packages > "$v/lib/python3.12/site-packages/py_global_overlay.pth"; \
    done && \
    src/megapose6d/venv/bin/pip install --no-deps -r /opt/demomotion/requirements/megapose.lock && \
    src/megapose6d/venv/bin/pip install --no-deps -e src/megapose6d && \
    src/third_party/pym3t/.venv/bin/pip install --no-deps src/third_party/pym3t

# FoundationPose mycpp (pybind11 iz venva: apt 2.11 nema numpy 2 i ruši se u cluster_poses)
RUN FP=src/third_party/FoundationPose V=/root/foundationpose/venv/bin/python && \
    mkdir -p $FP/mycpp/build && cd $FP/mycpp/build && \
    cmake .. -DPYTHON_EXECUTABLE=$V -Dpybind11_DIR=$($V -m pybind11 --cmakedir) && \
    make -j${COLCON_WORKERS} && \
    ln -sfn /root/arms_ws/src/third_party/FoundationPose /root/foundationpose/FoundationPose

# Frontend: dist/ je u imageu, pa gui_stack ne radi npm run build pri startu
RUN --mount=type=cache,target=/root/.npm \
    cd src/gui/arm_api2_gui && npm ci --legacy-peer-deps && npm run build

# ---------------------------------------------------------------- demomotion (finalni)
FROM build AS demomotion

# Modeli: $DEMOMOTION_MODELS se montira na /models (read-only).
# Symlinkovi s putanja koje kod očekuje:
#   sam/sam2.1_hiera_large.pt            /root/arm_perception/models/sam/
#   wilor/{wilor_final.ckpt,detector.pt} WiLoR/pretrained_models/ (yaml-ovi u toj mapi su u gitu)
#   (MANO je u gitu, WiLoR/mano_data/mano)
#   megapose/{coarse-rgb-*,refiner-*}    megapose6d/local_data/megapose-models
#   foundationpose/{2023-10-28-18-33-37,2024-01-11-20-02-45}   FoundationPose/weights
RUN W=/root/arms_ws/src/perception/WiLoR && \
    ln -sfn /models/foundationpose /root/arms_ws/src/third_party/FoundationPose/weights && \
    mkdir -p /root/arm_perception/models "$W/pretrained_models" "$W/mano_data" \
             /root/arms_ws/src/megapose6d/local_data && \
    ln -sfn /models/sam /root/arm_perception/models/sam && \
    ln -sfn /models/wilor/wilor_final.ckpt "$W/pretrained_models/wilor_final.ckpt" && \
    ln -sfn /models/wilor/detector.pt "$W/pretrained_models/detector.pt" && \
    ln -sfn /models/megapose /root/arms_ws/src/megapose6d/local_data/megapose-models

# Mjesta za volumene s podacima (compose), da postoje i bez njih
RUN mkdir -p /root/.ros/calibration /root/bags /root/arms_ws/diagnostics \
             /root/arm_perception/block_groups /var/log/arm_api2_gui /root/.config/arm_api2_gui && \
    touch /root/arms_ws/diagnostics/COLCON_IGNORE

COPY config/tmuxinator_gui_stack.yml /root/.tmuxinator/gui_stack.yml
COPY scripts/entrypoint.sh scripts/healthcheck.sh /opt/demomotion/
RUN chmod +x /opt/demomotion/*.sh && \
    printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '. /root/arms_ws/setup_ros_domain.sh' \
      'source /root/arms_ws/install/setup.bash' \
      'source /root/py_global/bin/activate' >> /root/.bashrc

# Monolit: GUI sam pokreće sve lokalno, ne preko docker compose/exec
ENV GUI_USE_CONTAINERS=0 \
    ARM_API2_LAUNCH_LOG_DIR=/var/log/arm_api2_gui/launch

LABEL org.opencontainers.image.title="demomotion" \
      org.opencontainers.image.source="https://github.com/CroboticSolutions/docker_files"
# Točni commitovi koji su ušli u image: /root/arms_ws/VERSIONS.yaml
EXPOSE 8080 8091 8181 8182 9094
HEALTHCHECK --interval=30s --timeout=8s --start-period=120s --retries=3 \
  CMD /opt/demomotion/healthcheck.sh
ENTRYPOINT ["/opt/demomotion/entrypoint.sh"]
CMD ["stack"]
