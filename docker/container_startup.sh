#!/bin/bash

# install ros2 dependencies
cd /home/ros2_ws
source /venv/bin/activate
rosdep init
rosdep update && rosdep install --from-paths src -yi

# build package
cd /home/ros2_ws
colcon build --symlink-install

# source workspace
source install/setup.bash

# get inside docker bash
bash