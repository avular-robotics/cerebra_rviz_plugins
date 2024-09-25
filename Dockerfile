FROM ros:humble

# add zenoh repository
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null

# install base dependencies
RUN apt-get update && apt-get install -y \
    ssh git git-lfs \
    zenoh-bridge-ros2dds=0.11.0 \
    ros-humble-rclcpp ros-humble-std-msgs ros-humble-rviz2 \
    ros-humble-rviz-common ros-humble-pluginlib \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    ros-humble-ament-cmake-clang-format \
    libqt5core5a libqt5gui5 libqt5widgets5 \
    && git lfs install

# create ROS workspace
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/src

# clone repository
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN --mount=type=ssh git clone git@github.com:avular-robotics/cerebra_rviz_plugins.git \
    && cd cerebra_rviz_plugins && git switch ESSENTIALS-2074-prepare-public-release
# RUN git clone https://github.com/avular-robotics/cerebra_rviz_plugins.git

# clone robot description
RUN git clone https://github.com/avular-robotics/avular_origin_description.git

# install packaged dependencies
WORKDIR /root/ws/src/cerebra_rviz_plugins
RUN apt-get install -y \
    ./interface_dependencies/autonomy-msgs_amd64_2.2.0.deb \
    ./interface_dependencies/origin-msgs_amd64_1.0.0.deb \
    ./interface_dependencies/cmake-avular_amd64_3.0.0.deb \
    ./interface_dependencies/ament-copyright-avular_amd64_3.0.0.deb

# build workspace
WORKDIR /root/ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

COPY entrypoint.sh /root/entrypoint.sh

# run entrypoint
ENTRYPOINT [ "/root/entrypoint.sh"]
CMD [ "127.0.0.1" ]
