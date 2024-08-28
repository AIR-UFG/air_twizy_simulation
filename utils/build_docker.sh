#!/bin/bash
# utils/build_docker.sh

# Check if the verification script exists
if [ ! -f "./utils/verify_submodules.sh" ]; then
    echo "Verification script not found! Exiting..."
    exit 1
fi

# Run the verification script
./utils/verify_submodules.sh

# If the verification script fails, exit
if [ $? -ne 0 ]; then
    echo "Submodule verification failed! Exiting..."
    exit 1
fi

# Build the Docker image
IMAGE_NAME="air-twizy-simulation-ufgsim" # Replace with your specific name

echo "Building Docker image with tag: $IMAGE_NAME"
docker build -t $IMAGE_NAME -f ./docker/Dockerfile .

if [ $? -eq 0 ]; then
    echo "Docker image built successfully with tag: $IMAGE_NAME"
else
    echo "Docker build failed! Exiting..."
    exit 1
fi
