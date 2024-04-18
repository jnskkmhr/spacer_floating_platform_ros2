#!/bin/bash

# install ros2 dependencies
cd /home/ros2_ws
rosdep update && rosdep install --from-paths src -yi
pip3 install cvxpy hydra-core omegaconf mujoco
cd ../thirdparty/rl_games && pip3 install -e .

# build package
cd /home/ros2_ws
colcon build

# install extra packages
apt-get update && apt-get install -y \
ros-$ROS_DISTRO-vrpn-mocap \
ros-$ROS_DISTRO-realsense2-camera \
ros-$ROS_DISTRO-realsense2-description

# source workspace
source install/setup.bash

# get inside docker bash
bash