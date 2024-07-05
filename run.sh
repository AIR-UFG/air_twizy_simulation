#!/bin/bash

export USERNAME=air

# Allow local connections to the X server for GUI applications in Docker
xhost +local:

# Load .env variables
set -a  # automatically export all variables
source .env
set +a

# Update DISPLAY and XAUTHORITY to ensure GUI works, needed for X11 forwarding

XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

export DISPLAY=:0
export XAUTHORITY=$XAUTH

# Create a volume to share files between the host and the container
export SHARED_FOLDER=$(pwd)/shared_folder
mkdir -p "$SHARED_FOLDER"

# Create a volume to develop ROS packages in the host and share them with the container(ONLY FOR DEVELOPMENT)
export HOST_FOLDER_PATH="$(pwd)/ros_packages" # Be sure to change this to the desired path
export CONTAINER_FOLDER_PATH="/home/$USERNAME/ros2_ws/src/" # Be sure to change this to the desired path

# Check if any argument is provided
if [ "$#" -gt 0 ]; then
    # Parse arguments in the form of KEY=value
    for arg in "$@"
    do
        key=$(echo $arg | cut -f1 -d=)
        value=$(echo $arg | cut -f2 -d=)
        
        # Validate the key and update the environment variable
        case "$key" in
            GPU|WORLD_NAME|RVIZ|PROJECTION|FOV_UP|FOV_DOWN|WIDTH|HEIGHT|WITH_VI)
                echo "Setting $key to $value"
                export "$key"="$value"
                ;;
            *)
                echo "Warning: Unknown setting '$key'. Ignored."
                ;;
        esac
    done
fi

# Set GPU-specific Docker Compose configurations
if [ "$GPU" = "true" ]; then
  if [ -z $(which nvidia-smi) ]; then
    echo "NVIDIA GPU support is requested but no NVIDIA GPU found. Running without GPU support."
    export GPU=false
    export NVIDIA_RUNTIME="runc"
  else
    echo "Enabling NVIDIA GPU support."
    export GPU=true
    export NVIDIA_RUNTIME="nvidia"
  fi
else
  echo "Running without NVIDIA GPU support."
  export GPU=false
  export NVIDIA_RUNTIME="runc"
fi

# Run Docker Compose
docker-compose up --build
