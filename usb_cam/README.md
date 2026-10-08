# usb_cam ROS2 Docker Image

Docker image with ROS2 and [CroboticSolutions' fork of `usb_cam`](https://github.com/CroboticSolutions/usb_cam)
(branch `apirsic/devel`) for plain USB/UVC webcams (RGB only, no depth/pointcloud --
that's what distinguishes this from the `realsense`/`luxonis` images).

This is the same fork/branch already used in
`../ros2/jazzy/mediapipe/Dockerfile`, kept consistent with the rest of the org's
tooling rather than the `ros-drivers/usb_cam` upstream or the
`ros-${ROS_DISTRO}-usb-cam` apt package (both of which are *also* used
elsewhere in this repo -- see `../ros2/jazzy/arm_api2_izzy_twist/Dockerfile`
and `../ros2/jazzy/mediapipe/Dockerfile` respectively -- so if you specifically
want the vanilla upstream instead, swap the git clone line in `Dockerfile`).

## Build

```bash
# Requires BuildKit for the SSH mount (private repo clone)
DOCKER_BUILDKIT=1 docker build --ssh default -t usb_cam_img:jazzy .
```

## Run

Use `run_docker.sh` for the first run (creates the container):

```bash
./run_docker.sh
```

Notes:
- `--privileged` and `--volume /dev:/dev` are required so the container can see the
  webcam's `/dev/videoN` device node(s) -- same pattern as `../realsense` and
  `../luxonis`.
- Find the right device node on the **host** first: `v4l2-ctl --list-devices` or
  `ls -la /dev/video*`. Update `video_device` in
  `to_copy/params_1.yaml` (baked into the image at
  `${USB_CAM_WS}/src/usb_cam/config/params_1.yaml`) to match, then rebuild --
  or mount a params file at runtime (`-v /path/to/params.yaml:/root/usb_cam_ws/src/usb_cam/config/params_1.yaml`)
  to avoid a rebuild per device index.

After the first run, use the usual workflow:
```bash
docker start -i usb_cam_cont      # resume the container
docker exec -it usb_cam_cont bash # open another shell into it
docker stop usb_cam_cont          # stop it
```

## Verifying the camera

Inside the container:

```bash
# Confirm the device is visible
v4l2-ctl --list-devices

# Launch the ROS2 camera node
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /root/usb_cam_ws/src/usb_cam/config/params_1.yaml

# From another shell/container (network_mode: host, same RMW_IMPLEMENTATION):
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view
```

## System Requirements

- Disk space: ~4GB
- A USB/UVC webcam connected via USB
