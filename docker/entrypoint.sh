#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# install extra packages
apt-get update && apt-get install -y \
ros-$ROS_DISTRO-vrpn-mocap \
ros-$ROS_DISTRO-realsense2-camera \
ros-$ROS_DISTRO-realsense2-description