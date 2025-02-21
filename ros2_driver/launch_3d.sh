#!/bin/bash
trap stop_ros EXIT

help()
{
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

CHECK_INTERVAL=5
TMUX_SESSION="ros_session"

if [[ "$sim" == "true" ]]; then
    MONITOR_IP="192.168.56.101"
else
    MONITOR_IP="192.168.0.10"
fi

stop_ros() {
    echo "[INFO] Stopping left over ROS processes (tmux session '$TMUX_SESSION')..."
    pkill -f octa_ros
    pkill -f dashboard_client
    pkill -f urscript
    pkill -f controller
    pkill -f moveit
    pkill -f ros
    #ros2 daemon stop
    #ros2 daemon start
    rm -f core*

    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
}

start_ros() {
    echo "[INFO] Starting ROS in tmux session '$TMUX_SESSION'..."
    stop_ros
    if [[ "$sim" == "true" ]]; then
        HOST="192.168.56.101"
        tmux new-session -d -s "$TMUX_SESSION" \
            "bash -ic 'source install/setup.bash; \
             ros2 launch octa_ros single_node_3d.py ur_type:=ur3e robot_ip:=$HOST headless_mode:=true'"
    else
        HOST="192.168.0.10"
        tmux new-session -d -s "$TMUX_SESSION" \
            "bash -ic 'source install/setup.bash; \
             ros2 launch octa_ros single_node_3d.py ur_type:=ur3e robot_ip:=$HOST headless_mode:=true reverse_ip:=192.168.0.2'"
    fi

    echo "[INFO] Tmux session '$TMUX_SESSION' created. You can attach with:"
    echo "       tmux attach -t $TMUX_SESSION"
    echo "[INFO] ROS Running"
}

echo "[INFO] Checking connectivity to $MONITOR_IP every $CHECK_INTERVAL seconds."
echo "[INFO] Press Ctrl+C to stop this monitor script."

ros_running=false
echo "[INFO] ROS is waiting for Robot to be online...."
while true
do
   if ping -c 1 -W 3 "$MONITOR_IP" &>/dev/null; then
       if ! $ros_running; then
           echo "[INFO] $MONITOR_IP is online. Starting ROS..."
           start_ros
           ros_running=true
       fi
   else
       if $ros_running; then
           echo "[INFO] $MONITOR_IP is offline. Stopping ROS..."
           stop_ros
           ros_running=false
           echo "[INFO] ROS is waiting for Robot to be online...."
       fi
   fi
   sleep "$CHECK_INTERVAL"
done
