#!/bin/bash
#tmux new -d "bash ur_driver/driver.sh"
source install/setup.bash
#ros2 launch octa_ros launch.py ur_type:=ur3e kinematic_params:=config/kinematics.yaml
#ros2 launch octa_ros combined.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true robot_controller:=scaled_joint_trajectory_controller kinematics_params_files:="config/ur_calibration.yaml" headless_mode:=true
ros2 launch octa_ros combined.py ur_type:=ur3e robot_ip:=192.168.56.101 headless_mode:=true
#kinematics_params_file:=config/ur_calibration.yaml
