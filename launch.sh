#!/bin/bash
trap stop_ros EXIT

source /opt/ros/jazzy/setup.bash
source install/setup.bash

help() {
	echo "Launch octa/oce ROS program"
	echo
	echo "Syntax: [-s|-d|-h]"
	echo "options:"
	echo "h     Print this Help."
	echo "s     Simulation"
	echo "d     Debug"
	echo
}

sim="false"
debug="false"
while getopts ":hsd" option; do
	case $option in
	h) # display Help
		help
		exit
		;;
	s) sim="true" ;;
	d) debug="true" ;;
	\?) # Invalid option
		echo "Error: Invalid option"
		exit
		;;
	esac
done

CHECK_INTERVAL=0.1
TMUX_SESSION="ros_driver_session"
init_start=true
HOST="192.168.0.2"

if [[ "$sim" == "true" ]]; then
	MONITOR_IP="192.168.56.101"
else
	MONITOR_IP="192.168.0.10"
fi

tmux_session_alive() {
	tmux has-session -t "$TMUX_SESSION" 2>/dev/null
}

stop_ros() {
	echo "[INFO] Stopping left over ROS processes (tmux session '$TMUX_SESSION')..."
	pkill -f octa_ros
	pkill -f dashboard_client
	pkill -f urscript
	pkill -f controller
	pkill -f moveit
	pkill -f ros
	rm -f core*
	tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
}

start_ros() {
	echo "[INFO] Starting ROS in tmux session '$TMUX_SESSION'..."
	stop_ros
	if [[ "$sim" == "true" ]]; then
		tmux new-session -d -s "$TMUX_SESSION" \
			"bash -ic 'source install/setup.bash; \
             ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$MONITOR_IP headless_mode:=true 2>&1 | tee /tmp/ros_launch.log'"
	else
		tmux new-session -d -s "$TMUX_SESSION" \
			"bash -ic 'source install/setup.bash; \
             ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$MONITOR_IP headless_mode:=true reverse_ip:=$HOST 2>&1 | tee /tmp/ros_launch.log'"
	fi

	echo "[INFO] Tmux session '$TMUX_SESSION' created. You can attach with:"
	echo "       tmux attach -t $TMUX_SESSION"
	echo "[INFO] ROS RUNNING"
}

check_labview_topic() {
	output=$(ros2 topic echo --once --timeout 0.2 /run_state std_msgs/msg/Bool 2>/dev/null)
	if [ -z "$output" ]; then
		echo ""
		return
	fi
	if [[ $output =~ "true" ]]; then
		echo "true"
	else
		echo "false"
	fi
}

if [[ $debug == "true" ]]; then
	trap - EXIT
	set -x
	stop_ros
	if [[ $sim == "true" ]]; then
		ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$MONITOR_IP headless_mode:=true
	else
		ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$MONITOR_IP headless_mode:=true reverse_ip:=$HOST
	fi
	stop_ros
	exit 0
fi

echo "[INFO] Checking connectivity to $MONITOR_IP every $CHECK_INTERVAL seconds."
echo "[INFO] Press Ctrl+C to stop this monitor script."
ros_running=false
echo "[INFO] ROS is waiting for Robot to be online...."
while true; do
	if ping -c 1 -W 3 "$MONITOR_IP" &>/dev/null; then
		if $init_start; then
			echo "[INFO] init_start=true"
			echo "[INFO] Robot is online. Starting ROS..."
			start_ros
			ros_running=true
			init_start=false
		fi
		new_state=$(check_labview_topic)
		if [[ "$new_state" == "true" ]]; then
			if ! $ros_running; then
				echo "[INFO] run_state=true"
				echo "[INFO] Robot is online. Starting ROS..."
				start_ros
				ros_running=true
			fi
		elif [[ "$new_state" == "false" ]]; then
			if $ros_running; then
				echo "[INFO] run_state=false"
				echo "[INFO] LabView deactivation. Stopping ROS..."
				stop_ros
				ros_running=false
			fi
		fi

		if $ros_running && ! tmux_session_alive; then
			echo "[WARN] ROS driver died - restarting..."
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
