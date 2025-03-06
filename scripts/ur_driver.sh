#!/bin/bash
help()
{
   # Display Help
   echo "Launch octa/oce ROS program"
   echo
   echo "Syntax: [-s|h]"
   echo "options:"
   echo "h     Print this Help."
   echo "s     Simulation"
   echo
}

while getopts ":hs" option; do
   case $option in
      h) # display Help
         help
         exit;;
      s) sim="true";;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done

source install/setup.bash
RVIZ=true

source install/setup.bash
if  [[ $sim == "true" ]]; then
	ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=$RVIZ initial_joint_controller:=scaled_joint_trajectory_controller kinematics_params_files:="./ur_calibration.yaml" headless_mode:=true
else
	ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.10 launch_rviz:=$RVIZ initial_joint_controller:=scaled_joint_trajectory_controller reverse_ip:=192.168.0.5 kinematics_params_files:="./ur_calibration.yaml" headless_mode:=true
fi



