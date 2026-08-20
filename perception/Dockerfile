# syntax=docker/dockerfile:1.7
# check=error=true

ARG CUDA_IMAGE=nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04@sha256:ac55d124da4882b497f732d8dfd9a702d5447a5f29d08d56da6f64f0a1eb34bc

FROM ${CUDA_IMAGE} AS ros_runtime_base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG ROS_APT_SOURCE_SHA256=0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1 \
    PERCEPTION_ROOT=/opt/arm_perception \
    PERCEPTION_WS=/opt/arm_perception/ws \
    VIRTUAL_ENV=/opt/arm_perception/venv \
    PATH=/opt/arm_perception/venv/bin:${PATH} \
    PYTHONPATH=/opt/arm_perception/venv/lib/python3.12/site-packages:/opt/arm_perception/ws/src/WiLoR

# Configure the official ROS repository using the signed ros2-apt-source package.
# Cache mounts speed up rebuilds without leaving apt indexes in the image layers.
# OS/ROS package versions intentionally follow the signed Noble/Jazzy repositories
# so scheduled --pull builds receive security fixes.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        locales \
        software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -fsSL --retry 5 --retry-delay 3 \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
        -o /tmp/ros2-apt-source.deb \
    && echo "${ROS_APT_SOURCE_SHA256}  /tmp/ros2-apt-source.deb" | sha256sum -c - \
    && apt-get install -y --no-install-recommends /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libegl1 \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libusb-1.0-0 \
        libxext6 \
        libxrender1 \
        python3-yaml \
        ros-${ROS_DISTRO}-ament-index-python \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-depth-image-proc \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-launch-ros \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-moveit-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ros2launch \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-sensor-msgs-py \
        ros-${ROS_DISTRO}-shape-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-visualization-msgs


FROM ros_runtime_base AS python_builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build the Python environment once. Runtime receives only the finished venv,
# not Git, compilers, pip caches, or other build tooling.
# Chumpy's setup.py imports pip, so only that package disables PEP 517 isolation.
# packaging>=24.2 is required by setuptools 80 when chumpy builds in the venv.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-dev \
        python3-pip \
        python3-venv

RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m venv --system-site-packages "${VIRTUAL_ENV}" \
    && python -m pip install --upgrade \
        pip==26.1 \
        setuptools==80.9.0 \
        wheel==0.42.0 \
        packaging==24.2 \
    && python -m pip install \
        --index-url https://download.pytorch.org/whl/cu128 \
        torch==2.10.0+cu128 \
        torchvision==0.25.0+cu128 \
    && python -m pip install \
        numpy==1.26.4 \
        scipy==1.17.1 \
    && python -m pip install --no-build-isolation \
        'chumpy @ git+https://github.com/mattloper/chumpy.git@580566eafc9ac68b2614b64d6f7aaa84eebb70da' \
    && python -m pip install \
        dill==0.4.0 \
        einops==0.8.1 \
        Flask==3.1.3 \
        Flask-Cors==6.0.2 \
        hydra-colorlog==1.2.0 \
        hydra-core==1.3.2 \
        hydra-submitit-launcher==1.2.0 \
        mediapipe==0.10.35 \
        numpy==1.26.4 \
        open3d==0.19.0 \
        opencv-contrib-python==4.11.0.86 \
        pandas==2.3.3 \
        Pillow==12.2.0 \
        pynput==1.8.1 \
        pyrender==0.1.45 \
        pyrootutils==1.0.4 \
        pytorch-lightning==2.6.1 \
        PyYAML==6.0.3 \
        rich==15.0.0 \
        scikit-image==0.26.0 \
        scipy==1.17.1 \
        segment-anything==1.0 \
        smplx==0.1.28 \
        timm==1.0.26 \
        transforms3d==0.4.2 \
        trimesh==4.11.4 \
        webdataset==1.0.2 \
        yacs==0.1.8 \
    && python -m pip check


FROM ros_runtime_base AS model_assets

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DOWNLOAD_MODELS=1
ARG SAM_MODEL_SHA256=ec2df62732614e57411cdcf32a23ffdf28910380d03139ee0f4fcbe91eb8c912
ARG WILOR_MODEL_SHA256=3e97aafc7dd08d883a4cc5a027df61fdb6fda6136dbd1319405413862ada6bb2
ARG MANO_MODEL_URL=https://drive.usercontent.google.com/download?id=1mAWwx53pkIefDxEcNuFZhcWUQ3_hi0JI&export=download&confirm=t
ARG MANO_MODEL_SHA256=45d60aa3b27ef9107a7afd4e00808f307fd91111e1cfa35afd5c4a62de264767

# This stage is independent of source branches: code updates do not invalidate
# the multi-gigabyte model layer. Keep access to images containing MANO limited
# to users covered by the applicable MANO license.
RUN --mount=type=cache,target=/var/cache/perception-models \
    set -eux; \
    install -d "${PERCEPTION_ROOT}/models/sam" \
        "${PERCEPTION_ROOT}/models/wilor" \
        "${PERCEPTION_ROOT}/models/mano"; \
    if [ "${DOWNLOAD_MODELS}" = "1" ]; then \
        sam_cache=/var/cache/perception-models/sam_vit_b_01ec64.pth; \
        if ! echo "${SAM_MODEL_SHA256}  ${sam_cache}" | sha256sum -c -; then \
            curl -fL --retry 5 --retry-delay 3 \
                https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth \
                -o "${sam_cache}.tmp"; \
            echo "${SAM_MODEL_SHA256}  ${sam_cache}.tmp" | sha256sum -c -; \
            mv "${sam_cache}.tmp" "${sam_cache}"; \
        fi; \
        install -m 0644 "${sam_cache}" \
            "${PERCEPTION_ROOT}/models/sam/sam_vit_b_01ec64.pth"; \
        wilor_cache=/var/cache/perception-models/wilor_final.ckpt; \
        if ! echo "${WILOR_MODEL_SHA256}  ${wilor_cache}" | sha256sum -c -; then \
            curl -fL --retry 5 --retry-delay 3 \
                https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/wilor_final.ckpt \
                -o "${wilor_cache}.tmp"; \
            echo "${WILOR_MODEL_SHA256}  ${wilor_cache}.tmp" | sha256sum -c -; \
            mv "${wilor_cache}.tmp" "${wilor_cache}"; \
        fi; \
        install -m 0644 "${wilor_cache}" \
            "${PERCEPTION_ROOT}/models/wilor/wilor_final.ckpt"; \
        mano_cache=/var/cache/perception-models/MANO_RIGHT.pkl; \
        if ! echo "${MANO_MODEL_SHA256}  ${mano_cache}" | sha256sum -c -; then \
            curl -fL --retry 5 --retry-delay 3 \
                "${MANO_MODEL_URL}" \
                -o "${mano_cache}.tmp"; \
            echo "${MANO_MODEL_SHA256}  ${mano_cache}.tmp" | sha256sum -c -; \
            mv "${mano_cache}.tmp" "${mano_cache}"; \
        fi; \
        install -m 0644 "${mano_cache}" \
            "${PERCEPTION_ROOT}/models/mano/MANO_RIGHT.pkl"; \
    fi


FROM ros_runtime_base AS workspace_builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG HPE_ROS_MSGS_BRANCH=apirsic/devel
ARG SAM_ROS2_BRANCH=multiple_ur_robots
ARG MP_ROS2_WRAPPER_BRANCH=apirsic/devel
ARG HANDEYE_BRANCH=apirsic/devel
ARG WILOR_BRANCH=main

# run_docker.sh resolves the current branch heads and passes their combined hash
# here. The value deliberately participates in this RUN instruction's cache key.
ARG SOURCE_REFRESH=manual

# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        openssh-client \
        python3-colcon-common-extensions \
        python3-pip \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-rosidl-default-generators

COPY --from=python_builder /opt/arm_perception/venv /opt/arm_perception/venv

RUN install -d -m 0700 /root/.ssh
COPY --chmod=0600 <<'KNOWN_HOSTS' /root/.ssh/known_hosts
github.com ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOMqqnkVzrm0SdG6UOoqKLsabgH5C9okWi0dh2l9GKJl
github.com ecdsa-sha2-nistp256 AAAAE2VjZHNhLXNoYTItbmlzdHAyNTYAAAAIbmlzdHAyNTYAAABBBEmKSENjQEezOmxkZMy7opKgwFB9nkt5YRrYMjNuG5N87uRgg6CLrbo5wAdT/y6v0mKV0U2w0WZ2YB/++Tpockg=
github.com ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCj7ndNxQowgcQnjshcLrqPEiiphnt+VTTvDP6mHBL9j1aNUkY4Ue1gvwnGLVlOhGeYrnZaMgRK6+PKCUXaDbC7qtbW8gIkhL7aGCsOr/C56SJMy/BCZfxd1nWzAOxSDPgVsmerOBYfNqltV9/hWCqBywINIR+5dIg6JTJ72pcEpEjcYgXkE2YEFXV1JHnsKgbLWNlhScqb2UmyRkQyytRLtL+38TGxkxCflmO+5Z8CSSNY7GidjMIZ7Q4zMjA2n1nGrlTDkzwDCsw+wqFPGQA179cnfGWOWRVruj16z6XyvxvjJwbz0wQZ75XK5tKSb7FNyeIEs4TT4jk+S4dhPeAUC5y+bDYirYgM4GC7uEnztnZyaVWQ7B381AK4Qdrwt51ZqExKbQpTUNn+EjqoTwvqNj4kqx5QUCI0ThS/YkOxJCXmPUWZbhjpCg56i+2aB6CmK2JGhn57K5mj0MNdBXA4/WnwH6XoPWJzK5Nyu2zB3nAZp+S5hpQs+p1vN1/wsjk=
KNOWN_HOSTS

WORKDIR /opt/arm_perception/ws

RUN --mount=type=ssh,required=true set -eux; \
    echo "Refreshing source set ${SOURCE_REFRESH}"; \
    install -d src; \
    clone_branch() { \
        clone_remote="$1"; clone_destination="$2"; clone_branch_name="$3"; \
        git clone --depth 1 --single-branch --branch "$clone_branch_name" \
            "$clone_remote" "$clone_destination"; \
        test "$(git -C "$clone_destination" branch --show-current)" = "$clone_branch_name"; \
    }; \
    clone_branch git@github.com:CroboticSolutions/hpe_ros_msgs.git \
        src/hpe_ros_msgs "${HPE_ROS_MSGS_BRANCH}"; \
    clone_branch git@github.com:CroboticSolutions/sam_ros2.git \
        src/sam_ros2 "${SAM_ROS2_BRANCH}"; \
    clone_branch git@github.com:CroboticSolutions/mp_ros2_wrapper.git \
        src/mp_ros2_wrapper "${MP_ROS2_WRAPPER_BRANCH}"; \
    clone_branch git@github.com:CroboticSolutions/ros2_handeye_calibration.git \
        src/ros2_handeye_calibration "${HANDEYE_BRANCH}"; \
    clone_branch https://github.com/rolpotamias/WiLoR.git \
        src/WiLoR "${WILOR_BRANCH}"; \
    { \
        for repository in hpe_ros_msgs sam_ros2 mp_ros2_wrapper ros2_handeye_calibration WiLoR; do \
            printf '%s %s %s\n' \
                "${repository}" \
                "$(git -C "src/${repository}" branch --show-current)" \
                "$(git -C "src/${repository}" rev-parse HEAD)"; \
        done; \
    } > SOURCE_REVISIONS; \
    sed -i \
        -e 's#/root/arms_ws/src/segment-anything/checkpoints/sam_vit_b_01ec64.pth#/opt/arm_perception/models/sam/sam_vit_b_01ec64.pth#g' \
        src/sam_ros2/sam_ros2/sam_ros_wrapper.py; \
    sed -i \
        -e 's#/root/py_global#/opt/arm_perception/venv#g' \
        -e 's#segment_anything_path = .*#segment_anything_path = venv_site_packages#' \
        -e 's#/root/arms_ws#/opt/arm_perception/ws#g' \
        src/sam_ros2/launch/sam_interactive.launch.py; \
    sed -i \
        -e 's#/root/arms_ws#/opt/arm_perception/ws#g' \
        src/sam_ros2/scripts/sam_api_bridge.py; \
    sed -i \
        -e 's#/root/arms_ws/src/mp_ros2_wrapper/models#/opt/arm_perception/ws/install/share/mp_wrapper_ros/models#g' \
        -e 's#/root/arms_ws#/opt/arm_perception/ws#g' \
        -e 's#/root/hamer/_DATA/data/mano#/opt/arm_perception/models/mano#g' \
        -e 's#/root/hamer#/opt/arm_perception/hamer#g' \
        -e 's#wilor_checkpoint: "pretrained_models/wilor_final.ckpt"#wilor_checkpoint: "/opt/arm_perception/models/wilor/wilor_final.ckpt"#g' \
        src/mp_ros2_wrapper/config/mp_config_oak_d_pro_w.yaml \
        src/mp_ros2_wrapper/config/mp_config_realsense_d435.yaml; \
    sed -i \
        -e 's#/root/arms_ws#/opt/arm_perception/ws#g' \
        -e 's#/root/hamer#/opt/arm_perception/hamer#g' \
        -e 's#/root/hpe_ws#/opt/arm_perception/hpe_ws#g' \
        src/mp_ros2_wrapper/mp_wrapper_ros/hamer_bridge.py; \
    sed -i \
        -e 's#/root/arms_ws#/opt/arm_perception/ws#g' \
        src/mp_ros2_wrapper/mp_wrapper_ros/wilor_bridge.py; \
    find src -mindepth 2 -maxdepth 2 -type d -name .git -prune -exec rm -rf '{}' +

RUN . "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon --log-base log build \
        --merge-install \
        --install-base install \
        --packages-select \
            hpe_ros_msgs \
            sam_ros2 \
            mp_wrapper_ros \
            hand_eye_calibration \
    && find \
        install/lib/sam_ros2 \
        install/lib/mp_wrapper_ros \
        install/lib/hand_eye_calibration \
        -maxdepth 1 -type f \
        -exec sed -i \
            -e '1s|^#!/usr/bin/python3$|#!/usr/bin/env python3|' \
            -e '1s|^#!python$|#!/usr/bin/env python3|' \
            '{}' +

# Import-level compatibility test. GPU/model validation is a runtime operation
# because Docker builds normally do not have GPU access and MANO is user-supplied.
RUN . "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && . install/setup.bash \
    && python -c 'import cv2, mediapipe, numpy, open3d, rclpy, torch; \
from cv_bridge import CvBridge; \
from hand_eye_calibration.charuco_detector import CharucoBoardDetector; \
from mp_wrapper_ros.hamer_mp_hamer_hand_node import HamerVisionNode; \
from mp_wrapper_ros.wilor_bridge import WiLoRBridge; \
from sam_ros2.sam_ros_wrapper import SAMROSWrapper; \
from segment_anything import sam_model_registry; \
from wilor.datasets.vitdet_dataset import ViTDetDataset; \
from wilor.models import load_wilor; \
assert hasattr(cv2, "aruco"); \
assert numpy.__version__.startswith("1.26."); \
assert "vit_b" in sam_model_registry'


FROM ros_runtime_base AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG APP_UID=1000
ARG APP_GID=1000

LABEL org.opencontainers.image.title="ARM perception" \
    org.opencontainers.image.description="ROS 2 Jazzy SAM, MediaPipe/WiLoR, and hand-eye calibration runtime" \
    org.opencontainers.image.vendor="Crobotic Solutions"

# Ubuntu 24.04 CUDA images already ship ubuntu:1000:1000. Reuse that
# identity when APP_UID/APP_GID collide instead of failing groupadd.
RUN set -eux; \
    existing_group="$(getent group "${APP_GID}" | cut -d: -f1 || true)"; \
    if [[ -z "${existing_group}" ]]; then \
        groupadd --gid "${APP_GID}" perception; \
    elif [[ "${existing_group}" != perception ]]; then \
        groupmod --new-name perception "${existing_group}"; \
    fi; \
    existing_user="$(getent passwd "${APP_UID}" | cut -d: -f1 || true)"; \
    if [[ -z "${existing_user}" ]]; then \
        useradd --no-log-init \
            --uid "${APP_UID}" \
            --gid "${APP_GID}" \
            --create-home \
            --home-dir /home/perception \
            --shell /bin/bash \
            perception; \
    elif [[ "${existing_user}" != perception ]]; then \
        usermod \
            --login perception \
            --home /home/perception \
            --move-home \
            --shell /bin/bash \
            --gid perception \
            "${existing_user}"; \
    fi; \
    install -d -o perception -g perception \
        /home/perception/.cache \
        /home/perception/.config/matplotlib \
        /home/perception/.ros \
        /opt/arm_perception/models/mano \
        /opt/arm_perception/ws/src/sam_ros2/scripts

COPY --from=python_builder --chown=perception:perception \
    /opt/arm_perception/venv /opt/arm_perception/venv
COPY --from=workspace_builder --chown=perception:perception \
    /opt/arm_perception/ws/install /opt/arm_perception/ws/install
COPY --from=workspace_builder --chown=perception:perception \
    /opt/arm_perception/ws/SOURCE_REVISIONS /opt/arm_perception/SOURCE_REVISIONS
COPY --from=workspace_builder --chown=perception:perception \
    /opt/arm_perception/ws/src/WiLoR /opt/arm_perception/ws/src/WiLoR
COPY --from=workspace_builder --chown=perception:perception \
    /opt/arm_perception/ws/src/sam_ros2/scripts/sam_api_bridge.py \
    /opt/arm_perception/ws/src/sam_ros2/scripts/sam_api_bridge.py
COPY --from=model_assets --chown=perception:perception \
    /opt/arm_perception/models /opt/arm_perception/models

COPY --chmod=0755 <<'ENTRYPOINT' /usr/local/bin/ros_entrypoint.sh
#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${PERCEPTION_WS}/install/setup.bash"

if [[ "${PERCEPTION_REQUIRE_MODELS:-0}" == "1" ]]; then
    required=(
        "${PERCEPTION_ROOT}/models/sam/sam_vit_b_01ec64.pth"
        "${PERCEPTION_ROOT}/models/wilor/wilor_final.ckpt"
        "${PERCEPTION_ROOT}/models/mano/MANO_RIGHT.pkl"
    )
    for asset in "${required[@]}"; do
        if [[ ! -f "${asset}" ]]; then
            echo "Missing required perception asset: ${asset}" >&2
            exit 1
        fi
    done
fi

if [[ "${PERCEPTION_REQUIRE_CUDA:-0}" == "1" ]]; then
    python -c 'import torch; assert torch.cuda.is_available(), "CUDA is unavailable; run the container with --gpus"'
fi

exec "$@"
ENTRYPOINT

ENV HOME=/home/perception \
    XDG_CACHE_HOME=/home/perception/.cache \
    MPLCONFIGDIR=/home/perception/.config/matplotlib

EXPOSE 5001

USER ${APP_UID}:${APP_GID}
WORKDIR /opt/arm_perception/ws

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
