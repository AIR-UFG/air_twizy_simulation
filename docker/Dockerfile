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
    ros-humble-gazebo-ros \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-velodyne* \
    ros-humble-ackermann* \
    ros-humble-can-msgs \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

RUN pip install transforms3d

# Switch to the non-root user
USER $USERNAME

# Set the working directory
WORKDIR /home/$USERNAME

RUN cd /home/$USERNAME/ && \
    git clone -b UFGSim https://github.com/AIR-UFG/colorcloud.git && \
    cd colorcloud && \
    pip install -e '.[dev]'

# Create the ROS2 workspace and copy the source files
RUN mkdir -p /home/$USERNAME/ros2_ws/src
COPY ./ros_packages/ /home/$USERNAME/ros2_ws/src/
COPY ./utils/ /home/$USERNAME/utils/

# Change ownership of the ROS2 workspace
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

# Copy Gazebo models to the appropriate directory
RUN mkdir -p /home/$USERNAME/.gazebo/models \
    && cp -r /home/$USERNAME/ros2_ws/src/vehicle_simulation_packages/air_sim/models/* /home/$USERNAME/.gazebo/models

# Copy the URDF file to the ROS installation directory
RUN sudo cp /home/$USERNAME/ros2_ws/src/vehicle_simulation_packages/air_description/urdf/sensors/VLP-16.urdf.xacro /opt/ros/humble/share/velodyne_description/urdf/

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
