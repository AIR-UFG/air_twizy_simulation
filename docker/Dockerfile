# Use the ROS Humble desktop full image as the base image
FROM osrf/ros:humble-desktop-full

# Set build arguments
ARG USERNAME=air
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user with sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

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
    && rm -rf /var/lib/apt/lists/*

# Switch to the non-root user
USER $USERNAME

# Set the working directory
WORKDIR /home/$USERNAME

# Create the ROS2 workspace and copy the source files
RUN mkdir -p /home/$USERNAME/ros2_ws/src
COPY ./ros_packages/ /home/$USERNAME/ros2_ws/src/
COPY ./utils/ /home/$USERNAME/utils/

# Change ownership of the ROS2 workspace
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$USERNAME/ros2_ws && colcon build --symlink-install"

# Create the entrypoint script
RUN echo "#!/bin/bash" > /home/$USERNAME/entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/entrypoint.sh \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/entrypoint.sh \
    && echo "exec \"\$@\"" >> /home/$USERNAME/entrypoint.sh \
    && sudo chmod +x /home/$USERNAME/entrypoint.sh \
    && echo "source /home/air/entrypoint.sh" >> /home/$USERNAME/.bashrc

# Set the entrypoint
ENTRYPOINT ["/home/air/entrypoint.sh"]

# Set the default command
CMD ["/bin/bash"]
