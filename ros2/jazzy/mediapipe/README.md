# ROS2 Jazzy - MediaPipe Human Pose Estimation Docker Image

Docker image for ROS2 Jazzy with MediaPipe integration for human pose estimation and tracking.

## Features

- **ROS2 Jazzy Desktop Full** on Ubuntu Noble
- **MediaPipe 0.10.14** - Google's ML framework for pose estimation
- **OpenCV** - Computer vision library
- **USB Camera Support** - ROS2 USB camera drivers
- **Custom HPE ROS2 Wrapper** - MediaPipe to ROS2 bridge
- **CycloneDDS** - Default RMW implementation

## Workspaces

**Primary workspace:** `/root/hpe_ws`

Contains:
- `mp_ros_wrapper` (branch: apirsic/devel) - MediaPipe ROS2 wrapper
- `hpe_ros_msgs` (branch: apirsic/devel) - Human pose estimation ROS2 messages
- `usb_cam` (branch: apirsic/devel) - USB camera driver

## MediaPipe Configuration

**Version:** 0.10.14
- Includes `framework.formats.landmark_pb2` module required by mp_ros_wrapper
- Compatible with numpy 1.26.4
- Works with opencv-python 4.8.1.78

## Quick Start

### Build the image

**Note:** This build requires SSH access to private repositories.

```bash
cd /home/toni/git/docker_files/ros2/jazzy/mediapipe
DOCKER_BUILDKIT=1 docker build --ssh default -t ros2_mp:jazzy .
```

### Prerequisites for building

1. SSH agent must be running with the appropriate keys:
```bash
eval $(ssh-agent)
ssh-add ~/.ssh/id_rsa  # or your private key
```

2. Verify SSH agent:
```bash
ssh-add -l
```

## Running the Container

### Basic run

```bash
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
xhost +local:docker
docker run -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name ros2_mp_cont \
  ros2_mp:jazzy \
  /bin/bash
```

### Run with GPU support

```bash
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
xhost +local:docker
docker run -it \
  --network host \
  --privileged \
  --gpus all \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name ros2_mp_cont \
  ros2_mp:jazzy \
  /bin/bash
```

## Usage Examples

### Launch MediaPipe ROS2 wrapper

Inside the container:

```bash
source /root/hpe_ws/install/setup.bash
ros2 run mp_ros_wrapper <node_name>
```

### Launch USB camera

```bash
source /root/hpe_ws/install/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

### View camera feed

```bash
source /root/hpe_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

### List human pose estimation topics

```bash
source /root/hpe_ws/install/setup.bash
ros2 topic list | grep hpe
```

## Environment Variables

Pre-configured environment variables:

- `ROS2_DISTRO=jazzy`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `TZ=Europe/Zagreb`
- `RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"`

## Installed ROS2 Packages

### Core packages:
- `ros-jazzy-desktop-full`
- `ros-jazzy-usb-cam`
- `ros-jazzy-image-transport`
- `ros-jazzy-image-transport-plugins`
- `ros-jazzy-cv-bridge`
- `ros-jazzy-rqt-image-view`
- `ros-jazzy-visualization-msgs`

### Python packages:
- `mediapipe==0.10.14`
- `numpy==1.26.4`
- `opencv-python==4.8.1.78`
- `python3-opencv` (system)

## Python Virtual Environment

A Python virtual environment is set up at `/root/py_global` with system site packages enabled.

Activated automatically in `.bashrc`.

## Camera Device Access

The container has access to `/dev` with privileged mode, allowing direct access to USB cameras and other devices.

Common camera devices:
- `/dev/video0` - First USB camera
- `/dev/video1` - Second USB camera

## Troubleshooting

**Build fails with SSH error:**
- Ensure SSH agent is running: `eval $(ssh-agent) && ssh-add`
- Verify you have access to CroboticSolutions private repositories
- Use `DOCKER_BUILDKIT=1` when building

**Camera not detected:**
- Check if camera is connected: `ls /dev/video*`
- Ensure container is running with `--privileged` flag
- Verify `/dev` volume is mounted

**X11 display error:**
- Run `xhost +local:docker` before starting container
- Ensure `DISPLAY` environment variable is set

**ImportError for mediapipe:**
- Source the workspace: `source /root/hpe_ws/install/setup.bash`
- Activate Python venv: `source /root/py_global/bin/activate`

**Numpy version conflict:**
- The image uses numpy 1.26.4 for compatibility with system matplotlib
- Don't upgrade numpy manually

## ROS2 Communication with Other Containers

This container uses `network: host` and `rmw_cyclonedds_cpp`, allowing it to communicate with other ROS2 containers on the same host.

Example: Communicate with UR robot driver container to create human-robot interaction applications.

## Docker Compose

This Dockerfile is used in the DemoMotion Docker Compose setup.

See: `/home/toni/git/docker_files/composers/demomotion/`
