#!/bin/sh
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.10 launch_rviz:=false robot_controller:=scaled_joint_trajectory_controller kinematics_params_files:="./ur_calibration.yaml" reverse_ip:=192.168.0.5 headless_mode:=true

#ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=false robot_controller:=scaled_joint_trajectory_controller kinematics_params_files:="./ur_calibration.yaml" headless_mode:=true activate_joint_controller:=true
