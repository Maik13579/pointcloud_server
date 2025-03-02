FROM ros:humble

#environment variables
ENV ROS_WS /root/ros2_ws
ENV DEBIAN_FRONTEND noninteractive

#ros2 DDS implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#use bash
SHELL ["/bin/bash", "-c"]

#source ros stuff
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

#install important stuff
RUN apt update \
    && apt install -y \
    libnanoflann-dev \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

#Copy only package.xml files to use caching and avoid reinstalling deps every time
COPY pointcloud_server/package.xml ${ROS_WS}/src/pointcloud_server/pointcloud_server/package.xml
COPY pointcloud_server_interfaces/package.xml ${ROS_WS}/src/pointcloud_server/pointcloud_server_interfaces/package.xml
WORKDIR ${ROS_WS}
RUN apt update \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src \
    && rm -rf /var/lib/apt/lists/*

# Build
COPY pointcloud_server ${ROS_WS}/src/pointcloud_server/pointcloud_server
COPY pointcloud_server_interfaces ${ROS_WS}/src/pointcloud_server/pointcloud_server_interfaces
WORKDIR ${ROS_WS}
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc
