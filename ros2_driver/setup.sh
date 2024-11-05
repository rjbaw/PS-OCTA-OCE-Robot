#!/bin/sh
sudo apt-get update
sudo apt-get install -y iproute2 iputils-ping traceroute vim tmux htop
sudo apt-get install -y ros-${ROS_DISTRO}-ros2-control
#sudo apt-get install -y ros-${ROS_DISTRO}-ur 

