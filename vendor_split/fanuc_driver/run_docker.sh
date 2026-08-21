#!/bin/bash

CONTAINER_NAME=fanuc_driver
IMAGE_NAME=fanuc_driver:latest

docker run \
  -it \
  --rm \
  --network host \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
