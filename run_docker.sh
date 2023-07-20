#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$CARLA_ROOT" ]; then
  echo "Error $CARLA_ROOT is empty. Set \$CARLA_ROOT as an environment variable first."
  exit 1
fi
echo "Using the CARLA version at '$CARLA_ROOT'"

${SCRIPT_DIR}/_utils/docker-gui \
    -it \
    --rm \
    --privileged \
    --ipc=host \
    --pid=host \
    --net=host \
    --volume=${CARLA_ROOT}/PythonAPI:/workspace/CARLA/PythonAPI:ro \
    --volume ${SCRIPT_DIR}/_config:/config \
    --volume=${SCRIPT_DIR}/ros2_demo:/workspace/ws/src/ros2_demo:rw \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastrtps-profile.xml \
    carla_ros2_demo:latest /bin/bash

