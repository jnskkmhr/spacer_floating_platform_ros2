#!/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd /home/ros2_ws
source /venv/bin/activate
exec "$@"