# Use an NVIDIA CUDA image as the base image with ROS Humble
FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04

# Set non-interactive mode for tzdata configuration
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Install ROS 2 Humble and Gazebo
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

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
    python3-colcon-common-extensions \ 
    ros-humble-gazebo-ros \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-velodyne* \
    ros-humble-ackermann* \
    ros-humble-can-msgs \
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
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/$USERNAME/ros2_ws && colcon build --symlink-install"

# Copy Gazebo models to the appropriate directory
RUN mkdir -p /home/$USERNAME/.gazebo/models \
    && cp -r /home/$USERNAME/ros2_ws/src/vehicle_simulation_packages/air_sim/models/* /home/$USERNAME/.gazebo/models

# Copy the URDF file to the ROS installation directory
RUN sudo cp /home/$USERNAME/ros2_ws/src/vehicle_simulation_packages/air_description/urdf/sensors/VLP-16.urdf.xacro /opt/ros/humble/share/velodyne_description/urdf/

# Create the entrypoint script
RUN echo "#!/bin/bash" > /home/$USERNAME/entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/entrypoint.sh \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/entrypoint.sh \
    && echo "exec \"\$@\"" >> /home/$USERNAME/entrypoint.sh \
    && sudo chmod +x /home/$USERNAME/entrypoint.sh \
    && echo "source /home/air/entrypoint.sh" >> /home/$USERNAME/.bashrc

# Set the entrypoint
ENTRYPOINT ["/home/air/entrypoint.sh"]

# Set the default command
CMD ["/bin/bash"]
