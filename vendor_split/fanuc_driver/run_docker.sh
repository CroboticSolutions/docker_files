#!/bin/bash

CONTAINER_NAME=fanuc_driver_cont
IMAGE_NAME=fanuc_driver_img:jazzy

docker run \
  -it \
  --rm \
  --network host \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
