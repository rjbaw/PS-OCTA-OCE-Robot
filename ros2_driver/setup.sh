#!/bin/sh
sudo apt-get update
sudo apt-get install -y rti-connext-dds-6.0.1 ros-${ROS_DISTRO}-rmw-connextdds iproute2 iputils-ping traceroute vim ros-${ROS_DISTRO}-ur
