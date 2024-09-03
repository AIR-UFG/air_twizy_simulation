# Use the ROS Humble desktop full image as the base image
FROM osrf/ros:humble-desktop-full

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    nano \
    wget \
    neovim \
    python3-pip \
    python3-rosdep \
    ros-humble-joint-state-publisher \
    ros-humble-velodyne* \
    ros-humble-ackermann* \
    ros-humble-can-msgs \
    ros-humble-teleop-twist-keyboard \
    can-utils \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /root/

# Create the ROS2 workspace and copy the source files
RUN mkdir -p /root/ros2_ws/src
COPY ./ros_packages/ /root/ros2_ws/src/
COPY ./utils/ /root/utils/

# Install cantools for can message decoding and visualization
RUN python3 -m pip install cantools
# Copy the streetdrone .dbc file
COPY ./utils/streetdrone_can.dbc /root/can/streetdrone_can.dbc

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"

RUN  echo "alias canmonitor='cantools monitor /root/can/streetdrone_can.dbc'" >> /root/.bashrc

# Create the entrypoint script
RUN echo "#!/bin/bash" > /root/entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/entrypoint.sh \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/entrypoint.sh \
    && echo "alias canmonitor='cantools monitor /root/can/streetdrone_can.dbc'" >> /root/entrypoint.sh \
    && echo "exec \"\$@\"" >> /root/entrypoint.sh \
    && sudo chmod +x /root/entrypoint.sh \
    && echo "source /root/entrypoint.sh" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
CMD ["/bin/bash"]
