#!/bin/sh
source "/opt/ros/${ROS_DISTRO}/setup.bash"
export RMW_IMPLEMENTATION=rmw_connextdds
export CONNEXTDDS_DIR=/opt/rti.com/rti_connext_dds-6.0.1
export NDDS_DISCOVERY_PEERS=$(ip route show | grep -i default | awk '{ print $3}')
export RTI_CONFIG_FILE=${HOME}/PS-OCTA-OCE-Labview-autofocus-Ren/dds_config.xml
#export RMW_CONNEXT_UDP_INTERFACE=eth0 #only for ddsmicro
export ROS_DOMAIN_ID=0
##source "/opt/rti.com/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash"
cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -
#source "/opt/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash"
export RTI_LICENSE_FILE="/root/home/uni_license.dat"
#source install/setup.bash

