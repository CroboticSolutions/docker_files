#!/bin/bash

CONTAINER_NAME=ur_driver
IMAGE_NAME=ur_driver:latest

docker run \
  -it \
  --rm \
  --network host \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
