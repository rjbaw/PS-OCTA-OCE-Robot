#!/bin/bash
tmux new -d "bash ur_driver/driver.sh"
source install/setup.bash
ros2 run octa_ros joint_state_listener
