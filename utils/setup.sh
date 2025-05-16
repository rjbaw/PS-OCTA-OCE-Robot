#!/bin/sh
sudo apt-get update
sudo apt-get install -y iproute2 iputils-ping traceroute vim tmux htop
sudo apt-get install -y ros-${ROS_DISTRO}-ros2-control
sudo apt-get install -y libglew-dev libglfw3-dev
sudo apt-get install -y ros-${ROS_DISTRO}-ur 
sudo apt-get install -y ros-${ROS_DISTRO}-ament-cmake-clang-format

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

