# ARM perception container

Production-oriented ROS 2 Jazzy GPU image for:

- SAM segmentation and grasp helpers (`sam_ros2`)
- MediaPipe + WiLoR hand perception (`mp_wrapper_ros`)
- ChArUco hand-eye calibration (`hand_eye_calibration`)

The image intentionally does **not** contain the GUI, WebRTC bridge, camera
drivers, robot drivers, `arm_api2`, or `arm_api2_msgs`. Camera and robot data are
expected over ROS 2/DDS from other host processes or containers.

## Prerequisites

- Docker with BuildKit/buildx
- NVIDIA Container Toolkit configured for Docker
- an SSH agent with access to the private Crobotic Solutions repositories

Verify SSH access before building:

```bash
ssh-add -l
ssh -T git@github.com
```

## Build and run

The helper resolves the current remote heads of all configured branches and
passes a combined fingerprint to Docker. This invalidates the source layer when
a branch advances while preserving the expensive Python and model caches.

```bash
cd /root/docker/perception
./run_docker.sh
```

With no arguments, the container opens a ROS-sourced Bash shell. Supply a
command to run a specific perception workload:

```bash
# OAK-D camera topics are expected from another ROS process/container.
./run_docker.sh \
  ros2 launch mp_wrapper_ros hamer_mp_hamer_oak_d_pro_w.launch.py

# Interactive SAM stack.
./run_docker.sh \
  ros2 launch sam_ros2 sam_interactive.launch.py robot:=piper

# Hand-eye calibration; camera and robot TF must already be available.
./run_docker.sh \
  ros2 launch hand_eye_calibration calibration.launch.py
```

The SAM HTTP bridge currently listens on port `5001`. Host networking is used
for reliable ROS 2 discovery, so `EXPOSE` is informational.

## MANO model

The default build downloads `MANO_RIGHT.pkl` from the configured Google Drive
location together with the SAM and WiLoR checkpoints. Its SHA-256 checksum is
verified before it is copied into the image. Access to the Drive file and built
image must be limited to users covered by the applicable MANO license.

To override the model embedded in the image, point `MANO_DIR` at a directory
containing your licensed copy:

```bash
MANO_DIR=/secure/path/to/mano ./run_docker.sh \
  ros2 launch mp_wrapper_ros hamer_mp_hamer_oak_d_pro_w.launch.py
```

The directory is mounted read-only at `/opt/arm_perception/models/mano`. When
`MANO_DIR` is supplied, startup checks that SAM, WiLoR, MANO, and CUDA are all
available.

## Useful options

```bash
# Build without downloading SAM, WiLoR, or MANO model assets.
DOWNLOAD_MODELS=0 BUILD_ONLY=1 ./run_docker.sh

# Skip an already completed image build.
SKIP_BUILD=1 ./run_docker.sh bash

# Override image/container names or ROS domain.
IMAGE_NAME=registry.example/arm-perception:dev \
CONTAINER_NAME=arm-perception-dev \
ROS_DOMAIN_ID=42 \
./run_docker.sh
```

Source branches can also be overridden with `HPE_ROS_MSGS_BRANCH`,
`SAM_ROS2_BRANCH`, `MP_ROS2_WRAPPER_BRANCH`, `HANDEYE_BRANCH`, and
`WILOR_BRANCH`.

The exact branch heads used for an image are recorded inside it at:

```text
/opt/arm_perception/SOURCE_REVISIONS
```

## Image design

- pinned NVIDIA CUDA base image digest
- multi-stage build; compiler, Git, SSH, and colcon stay out of runtime
- exact direct Python dependency versions and pinned Chumpy revision
- verified ROS apt-source package and model checksums
- BuildKit SSH forwarding; no private key is copied into the image
- verified GitHub host keys instead of build-time `ssh-keyscan`
- non-root runtime user
- source-aware cache invalidation and independent model cache
- import/ABI smoke test during the build

This is a multi-purpose runtime image, so it deliberately has no global
`HEALTHCHECK`: a meaningful health probe depends on the selected ROS launch
command. A production Compose/systemd deployment should supply the command,
restart policy, and a workload-specific ROS topic/service health probe.

Private Python source remains inspectable by anyone who can pull the image.
Store the built image in an access-controlled registry.
