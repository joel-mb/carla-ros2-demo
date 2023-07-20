#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -fr ${SCRIPT_DIR}/.tmp

echo "Creating a temporary folder"
mkdir -p ${SCRIPT_DIR}/.tmp

echo "Copying the ros2 workspace into the folder"
cp -fr ${SCRIPT_DIR}/ros2_demo/ ${SCRIPT_DIR}/.tmp/ros2_demo
cp -fr ${SCRIPT_DIR}/_submodules/carla_msgs/ ${SCRIPT_DIR}/.tmp/carla_msgs

cp ${SCRIPT_DIR}/entrypoint.sh ${SCRIPT_DIR}/.tmp/entrypoint.sh

echo "Building the docker"
docker build --force-rm -t carla_ros2_demo -f Dockerfile ${SCRIPT_DIR}/.

echo "Removing the temporary folder"
rm -fr ${SCRIPT_DIR}/.tmp
