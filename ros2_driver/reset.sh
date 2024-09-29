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

sim="false"
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

pkill ros
rm -f core*
source ur_driver/install/setup.bash
source install/setup.bash
if  [[ $sim == "true" ]]; then
	ros2 launch octa_ros reset.py ur_type:=ur3e robot_ip:=192.168.56.101 headless_mode:=true
else
	ros2 launch octa_ros reset.py ur_type:=ur3e robot_ip:=192.168.0.10 headless_mode:=true reverse_ip:=192.168.0.5

fi
