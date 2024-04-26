#!/bin/bash
xhost +
# recursiely find path to ros2_ws
WORKSPACE="$(dirname "${PWD}")"
WORKSPACE="$(dirname "${WORKSPACE}")"
docker run --name fp-ros2-container -it --gpus all \
-e "ACCEPT_EULA=Y" --privileged --network=host \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v $HOME/.Xauthority:/root/.Xauthority \
-v /dev/:/dev/ \
-v $WORKSPACE:/home/ros2_ws \
fp-ros2:latest /home/ros2_ws/src/spacer_floating_platform_ros2/docker/container_startup.sh