#!/bin/bash

# MuJoCo Simulator Docker Runner Script

# Build image if it doesn't exist
IMAGE_NAME="ros2-mujoco:jazzy"
CONTAINER_NAME="ros2_mujoco_dev"

echo "Building MuJoCo Docker image..."
docker-compose build --no-cache

echo "Running MuJoCo Docker container..."
docker-compose run --rm mujoco bash

echo "Container stopped."
