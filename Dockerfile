#ARG ROS_DISTRO="foxy"
#ARG ROS_DISTRO="iron"
ARG ROS_DISTRO="humble"

FROM osrf/ros:$ROS_DISTRO-desktop

WORKDIR /workspace

RUN apt-get update \
    && apt-get install -y --no-install-recommends \ 
        libpng-dev libtiff5-dev libjpeg-dev \
    && apt-get install -y ros-$ROS_DISTRO-imu-tools \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3 \
    && pip3 install transforms3d

ENV CARLA_ROOT "/workspace/CARLA"
ENV ROS2_WS "/workspace/ws"

COPY .tmp/ros2_demo "${ROS2_WS}/src/ros2_demo"
COPY .tmp/carla_msgs "${ROS2_WS}/src/carla_msgs"

COPY .tmp/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]
