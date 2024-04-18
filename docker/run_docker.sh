#!/bin/bash
xhost +
PACKAGE_NAME="$(basename "${PWD}")"
docker run --name rans-ros-container -it --gpus all \
-e "ACCEPT_EULA=Y" --rm --privileged --network=host \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $PWD:/home/ros2_ws/src/$PACKAGE_NAME \
rans-ros:latest /home/ros2_ws/src/spacer_floating_platform_ros2/docker/container_startup.sh