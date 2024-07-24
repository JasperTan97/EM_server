#!/bin/bash

# Navigate to the docker_base directory
cd docker_base

# Build the Docker image with the tag 'ros2_humble'
docker build -t ros2_humble .

# Navigate back to the previous directory
cd ..

# Build the main docker container
docker-compose build