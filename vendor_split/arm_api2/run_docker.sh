#!/bin/bash

CONTAINER_NAME=arm_api2_cont
IMAGE_NAME=arm_api2:latest

docker run \
  -it \
  --rm \
  --network host \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
