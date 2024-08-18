#!/bin/bash

# Navigate to the docker_base directory
cd docker_base

# Build the Docker image with the tag 'ros2_humble'
docker build -f Dockerfile.base -t ros2_humble .

# Build the Docker image with the tag 'custom-gazebo-ros2'
# docker build -f Dockerfile.gazebo -t custom-gazebo-ros2 .

# Navigate back to the previous directory
cd ..

# Build the main docker container
docker-compose build

# # create a docker network
# docker network create ros2_network