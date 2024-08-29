#!/bin/bash
tmux new -d "bash ur_driver/driver.sh"
source install/setup.bash
ros2 launch octa_ros launch.py ur_type:=ur3e
