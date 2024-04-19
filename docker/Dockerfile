# for x86
FROM osrf/ros:foxy-desktop
# for jetson
FROM dustynv/ros:foxy-desktop-l4t-r32.7.1

# Standard package
RUN apt-get update && apt-get install -y git vim wget byobu zip unzip

# Python dependencies
RUN apt-get install -y python3-tk

# ROS build dependencies
RUN apt-get update && apt-get install -y python3-colcon-common-extensions python3-rosdep

# Other thirdparty packages
WORKDIR /home/thirdparty
RUN git clone https://github.com/AntoineRichard/rl_games.git

WORKDIR /home