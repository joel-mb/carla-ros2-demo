#!/bin/bash
set -e

PYTHON_VERSION=$(python3 -c "import sys; print('{}.{}'.format(sys.version_info[0], sys.version_info[1]))")
export PYTHONPATH=$PYTHONPATH:"${CARLA_ROOT}/PythonAPI/carla/dist/$(ls ${CARLA_ROOT}/PythonAPI/carla/dist | grep py${PYTHON_VERSION})"
export PYTHONPATH=$PYTHONPATH:"${CARLA_ROOT}/PythonAPI/carla"

source "/opt/ros/$ROS_DISTRO/setup.bash"

exec "$@"
