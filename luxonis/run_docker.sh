#!/bin/bash

CONTAINER_NAME=ros2_oak_sr_cont
IMAGE_NAME=ros2_oak_sr:humble

docker run \
  -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $CONTAINER_NAME \
  $IMAGE_NAME \
  /bin/bash
