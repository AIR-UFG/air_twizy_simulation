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
    ros-humble-gazebo-ros \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-velodyne* \
    ros-humble-ackermann* \
    ros-humble-can-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /root/

# Create the ROS2 workspace and copy the source files
RUN mkdir -p /root/ros2_ws/src
COPY ./ros_packages/ /root/ros2_ws/src/
COPY ./utils/ /root/utils/

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"

# Copy Gazebo models to the appropriate directory
RUN mkdir -p /root/.gazebo/models \
    && cp -r /root/ros2_ws/src/vehicle_simulation_packages/air_sim/models/* /root/.gazebo/models

# Copy the URDF file to the ROS installation directory
RUN sudo cp /root/ros2_ws/src/vehicle_simulation_packages/air_description/urdf/sensors/VLP-16.urdf.xacro /opt/ros/humble/share/velodyne_description/urdf/

# Create the entrypoint script
RUN echo "#!/bin/bash" > /root/entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/entrypoint.sh \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/entrypoint.sh \
    && echo "exec \"\$@\"" >> /root/entrypoint.sh \
    && sudo chmod +x /root/entrypoint.sh \
    && echo "source /root/entrypoint.sh" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
CMD ["/bin/bash"]
