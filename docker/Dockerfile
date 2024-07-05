# docker build -t air-twizy-simulation -f docker/Dockerfile .

FROM osrf/ros:humble-desktop-full

ARG USERNAME=air
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt update && apt install -y \
    git \
    nano \
    wget \
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

USER $USERNAME

WORKDIR /home/$USERNAME

RUN mkdir -p /home/$USERNAME/ros2_ws/src

COPY ./ros_packages/ /home/$USERNAME/ros2_ws/src/

RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$USERNAME/ros2_ws && colcon build --symlink-install"

RUN mkdir -p /home/$USERNAME/.gazebo/models && cp -r /home/air/ros2_ws/src/vehicle_simulation_packages/air_sim/models/* /home/$USERNAME/.gazebo/models
 
RUN /bin/bash -c "sudo cp /home/air/ros2_ws/src/vehicle_simulation_packages/air_description/urdf/sensors/VLP-16.urdf.xacro /opt/ros/humble/share/velodyne_description/urdf/"

RUN echo "#!/bin/bash" >> /home/$USERNAME/entrypoint.sh && \
    echo "set -e" >> /home/$USERNAME/entrypoint.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/entrypoint.sh && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/entrypoint.sh && \
    echo "exec \"\$@\"" >> /home/$USERNAME/entrypoint.sh && \
    echo "source /home/$USERNAME/entrypoint.sh" >> /home/$USERNAME/.bashrc && \
    /bin/bash -c "sudo chmod +x /home/$USERNAME/entrypoint.sh"
    
ENTRYPOINT ["./entrypoint.sh"]

CMD ["/bin/bash"]


