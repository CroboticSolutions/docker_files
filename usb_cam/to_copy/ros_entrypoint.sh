#!/bin/bash
# Autostarts the usb_cam driver node in the background, then hands off to
# whatever CMD/`docker compose command:` was passed (bash, by default) --
# same pattern as perception_cont's SAM autostart. Gate/tune via env vars so
# a rebuild isn't needed to disable it or change the params file/delay.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${USB_CAM_WS}/install/setup.bash

if [[ "${USB_CAM_AUTOSTART:-1}" == "1" ]]; then
    (
        sleep "${USB_CAM_AUTOSTART_DELAY_SEC:-3}"
        ros2 run usb_cam usb_cam_node_exe --ros-args --params-file \
            "${USB_CAM_AUTOSTART_PARAMS_FILE:-${USB_CAM_WS}/src/usb_cam/config/params_1.yaml}" \
            > /tmp/usb_cam_autostart.log 2>&1
    ) &
fi

exec "$@"
