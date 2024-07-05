#!/bin/bash

# Check if an image name was provided
# if [ "$#" -ne 1 ]; then
#     echo "Usage: $0 <image-name>"
#     exit 1
# fi

IMAGE_NAME=air-twizy-simulation:latest
USERNAME=air

# Allow local connections to the X server for GUI applications in Docker
xhost +local:
# xhost +

# Setup for X11 forwarding to enable GUI
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Create a volume to share files between the host and the container

HOST_FOLDER_PATH="/home/matteus/victor/lume/tmp/organizing_it/air_twizy_simulation/ros_packages" # Be sure to change this to the desired path
CONTAINER_FOLDER_PATH="/home/$USERNAME/ros2_ws/src/" # Be sure to change this to the desired path

# Run the Docker container with the selected image and configurations for GUI applications
docker run -it \
    --user root \
    --name air_container \
    --privileged \
    --network=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_LOCALHOST_ONLY=1" \
    --env="ROS_DOMAIN_ID=91" \
    --env="TERM=xterm-256color" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume "$HOST_FOLDER_PATH:$CONTAINER_FOLDER_PATH:rw" \
    $IMAGE_NAME

# If NVIDIA GPU is available, uncomment the following lines to enable GPU acceleration:
    # --runtime nvidia \
    # --env="NVIDIA_VISIBLE_DEVICES=all" \
    # --env="NVIDIA_DRIVER_CAPABILITIES=all" \

# If you want to pass volumes

    # --volume "/dev:/dev" \
    # --volume "$HOST_FOLDER_PATH:$CONTAINER_FOLDER_PATH" \

#   --env="ROS_LOCALHOST_ONLY=1" \
#   --env="ROS_DOMAIN_ID=91" \

    # --env="NVIDIA_VISIBLE_DEVICES=all" \
    # --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    # --runtime nvidia \