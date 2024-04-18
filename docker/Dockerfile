FROM nnvidiajetson/l4t-ros2-foxy

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