#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag record /oct_image /labview_data /cancel_current_action --qos-profile-overrides-path config/bag_qos.yaml -o bags/$1
