#!/bin/bash

CONTAINER_NAME=piper_driver
IMAGE_NAME=piper_driver:latest

# Real hardware needs --privileged + /dev for the USB-CAN adapter (can0).
# Drop --privileged/-v /dev:/dev for simulation-only use.
docker run \
  -it \
  --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
