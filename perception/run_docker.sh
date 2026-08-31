#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

image_name=perception_img:jazzy
container_name=perception_cont
download_models=${DOWNLOAD_MODELS:-1}
ros_domain_id=${ROS_DOMAIN_ID:-0}

hpe_branch=${HPE_ROS_MSGS_BRANCH:-apirsic/devel}
sam_branch=${SAM_ROS2_BRANCH:-multiple_ur_robots}
mp_branch=${MP_ROS2_WRAPPER_BRANCH:-apirsic/devel}
handeye_branch=${HANDEYE_BRANCH:-apirsic/devel}
wilor_branch=${WILOR_BRANCH:-main}

command -v docker >/dev/null || {
    echo "Docker is not installed or not available on PATH." >&2
    exit 1
}

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
    command -v git >/dev/null || {
        echo "Git is required to resolve branch heads." >&2
        exit 1
    }
    if ! ssh-add -l >/dev/null 2>&1; then
        echo "No SSH key is loaded. Start ssh-agent and run ssh-add first." >&2
        exit 1
    fi

    resolve_branch() {
        local remote=$1
        local branch=$2
        local revision

        revision=$(git ls-remote --exit-code --refs \
            "${remote}" "refs/heads/${branch}" | awk 'NR == 1 {print $1}')
        if [[ -z "${revision}" ]]; then
            echo "Branch ${branch} was not found in ${remote}." >&2
            return 1
        fi
        printf '%s\n' "${revision}"
    }

    hpe_revision=$(resolve_branch \
        git@github.com:CroboticSolutions/hpe_ros_msgs.git "${hpe_branch}")
    sam_revision=$(resolve_branch \
        git@github.com:CroboticSolutions/sam_ros2.git "${sam_branch}")
    mp_revision=$(resolve_branch \
        git@github.com:CroboticSolutions/mp_ros2_wrapper.git "${mp_branch}")
    handeye_revision=$(resolve_branch \
        git@github.com:CroboticSolutions/ros2_handeye_calibration.git \
        "${handeye_branch}")
    wilor_revision=$(resolve_branch \
        https://github.com/rolpotamias/WiLoR.git "${wilor_branch}")

    source_state=$(printf '%s\n' \
        "hpe_ros_msgs ${hpe_branch} ${hpe_revision}" \
        "sam_ros2 ${sam_branch} ${sam_revision}" \
        "mp_ros2_wrapper ${mp_branch} ${mp_revision}" \
        "ros2_handeye_calibration ${handeye_branch} ${handeye_revision}" \
        "WiLoR ${wilor_branch} ${wilor_revision}")

    source_refresh=$(printf '%s' "${source_state}" | sha256sum | awk '{print $1}')

    docker buildx build \
        --pull \
        --ssh default \
        --build-arg "DOWNLOAD_MODELS=${download_models}" \
        --build-arg "HANDEYE_BRANCH=${handeye_branch}" \
        --build-arg "HPE_ROS_MSGS_BRANCH=${hpe_branch}" \
        --build-arg "MP_ROS2_WRAPPER_BRANCH=${mp_branch}" \
        --build-arg "SAM_ROS2_BRANCH=${sam_branch}" \
        --build-arg "SOURCE_REFRESH=${source_refresh}" \
        --build-arg "WILOR_BRANCH=${wilor_branch}" \
        --load \
        --tag "${image_name}" \
        "${script_dir}"
fi

if [[ "${BUILD_ONLY:-0}" == "1" ]]; then
    exit 0
fi

docker_args=(
    run
    -d
    -it
    --name "${container_name}"
    --gpus all
    --network host
    --ipc host
    --env "ROS_DOMAIN_ID=${ros_domain_id}"
    --env "PYNPUT_BACKEND=${PYNPUT_BACKEND:-dummy}"
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    docker_args+=(
        --env "DISPLAY=${DISPLAY}"
        --volume /tmp/.X11-unix:/tmp/.X11-unix:ro
    )
fi

if [[ -n "${MANO_DIR:-}" ]]; then
    mano_dir=$(cd -- "${MANO_DIR}" && pwd)
    if [[ ! -f "${mano_dir}/MANO_RIGHT.pkl" ]]; then
        echo "MANO_RIGHT.pkl was not found in MANO_DIR=${mano_dir}." >&2
        exit 1
    fi
    docker_args+=(
        --env PERCEPTION_REQUIRE_CUDA=1
        --env PERCEPTION_REQUIRE_MODELS=1
        --volume "${mano_dir}:/opt/arm_perception/models/mano:ro"
    )
fi

if [[ $# -eq 0 ]]; then
    set -- bash
fi

exec docker "${docker_args[@]}" "${image_name}" "$@"