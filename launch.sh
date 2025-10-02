#!/bin/bash
trap stop_ros EXIT

source /opt/ros/jazzy/setup.bash
source install/setup.bash || true
source /workspace/app/setup.bash || true

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
	h)
		help
		exit
		;;
	s) sim="true" ;;
	d) debug="true" ;;
	\?)
		echo "Error: Invalid option"
		exit
		;;
	esac
done

LOG_DIR="${LOG_DIR:-$PWD/logs}"
PRUNE_LOGS_DAYS="${PRUNE_LOGS_DAYS:-7}"
mkdir -p "$LOG_DIR"

prune_logs() {
	find "$LOG_DIR" -mindepth 1 -mtime +"$PRUNE_LOGS_DAYS" -exec rm -rf {} + 2>/dev/null || true
}
prune_logs

CHECK_INTERVAL="${CHECK_INTERVAL:-0.3}"
PING_TIMEOUT="${PING_TIMEOUT:-3}"
MAX_RETRIES="${MAX_RETRIES:-10}"
HOST_IP="${HOST_IP:-192.168.0.2}"
RUN_STATE_TIMEOUT="${RUN_STATE_TIMEOUT:-0.25s}"

TMUX_SESSION="ros_driver_session"
init_start=true

if [[ -z "${ROBOT_IP:-}" ]]; then
	if [[ "$sim" == "true" ]]; then
		ROBOT_IP="192.168.56.101"
	else
		ROBOT_IP="192.168.0.10"
	fi
fi

session_alive() { tmux has-session -t "$TMUX_SESSION" 2>/dev/null; }
pane_dead() { tmux display-message -p -t "$TMUX_SESSION" "#{pane_dead}" 2>/dev/null | grep -q '^1$'; }

stop_ros() {
	echo "[INFO] Stopping ROS processes (tmux session '$TMUX_SESSION')..."
	pkill -f octa_ros
	pkill -f ur_robot_driver || true
	pkill -f dashboard_client || true
	pkill -f controller_stopper_node || true
	pkill -f urscript_interface || true
	rm -f /tmp/launch_params_* core*
	tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
}

start_ros() {
	echo "[INFO] Starting ROS in tmux session '$TMUX_SESSION'..."
	stop_ros
	if [[ "$sim" == "true" ]]; then
		tmux new-session -d -s "$TMUX_SESSION" \
			"bash -lc 'set -e; export RCUTILS_LOGGING_DIRECTORY=\"$LOG_DIR\"; \
              [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash; \
              [ -f /workspace/app/install/setup.bash ] && source /workspace/app/install/setup.bash; \
              cd /workspace/app 2>/dev/null || true; \
              ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$ROBOT_IP headless_mode:=true'"
	else
		tmux new-session -d -s "$TMUX_SESSION" \
			"bash -lc 'set -e; export RCUTILS_LOGGING_DIRECTORY=\"$LOG_DIR\"; \
              [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash; \
              [ -f /workspace/app/install/setup.bash ] && source /workspace/app/install/setup.bash; \
              cd /workspace/app 2>/dev/null || true; \
              ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$ROBOT_IP headless_mode:=true reverse_ip:=$HOST_IP'"
	fi
	tmux set-window-option -t "$TMUX_SESSION" remain-on-exit on
	tmux set-option -t "$TMUX_SESSION" history-limit 50000
	echo "[INFO] Tmux session '$TMUX_SESSION' created. Attach with: tmux attach -t $TMUX_SESSION"
	echo "[INFO] ROS RUNNING"
}

save_tail() {
	local ts crash_log
	ts=$(date +%Y%m%d-%H%M%S)
	crash_log="$LOG_DIR/ros_crash_$ts.log"
	if session_alive; then
		tmux capture-pane -t "$TMUX_SESSION" -p -S -300 >"$crash_log" 2>/dev/null || true
		echo "[INFO] Saved last 300 lines to $crash_log"
		sed -e 's/^/[ROS] /' "$crash_log" || true
	else
		echo "[WARN] tmux session not found; nothing to capture"
	fi
}

check_labview_topic() {
	# timeout 0.25s ros2 topic echo -n 1 -q --field data /run_state 2>/dev/null
	if ! timeout "$RUN_STATE_TIMEOUT" ros2 run octa_ros run_state_listener \
		>/tmp/run_state.out 2>/dev/null; then
		printf '\n'
		return
	fi
	cat /tmp/run_state.out
}

if [[ $debug == "true" ]]; then
	trap - EXIT
	set -x
	stop_ros
	if [[ $sim == "true" ]]; then
		ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$ROBOT_IP headless_mode:=true
	else
		ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$ROBOT_IP headless_mode:=true reverse_ip:=$HOST_IP
	fi
	stop_ros
	exit 0
fi

echo "[INFO] Logs dir: $LOG_DIR"
echo "[INFO] Checking connectivity to $ROBOT_IP every $CHECK_INTERVAL seconds."
echo "[INFO] Press Ctrl+C to stop this monitor script."
ros_running=false
fails=0
echo "[INFO] ROS is waiting for Robot to be online...."
while true; do
	if ping -c 1 -W "$PING_TIMEOUT" "$ROBOT_IP" &>/dev/null; then
		fails=0
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

		if $ros_running && { pane_dead || ! session_alive; }; then
			echo "[WARN] ROS driver died"
			save_tail
			echo "[WARN] Restarting ROS..."
			start_ros
			ros_running=true
		fi

	else
		((fails += 1))
		if $ros_running; then
			echo "[INFO] $ROBOT_IP is offline. Stopping ROS..."
			stop_ros
			ros_running=false
			echo "[INFO] ROS is waiting for Robot to be online...."
		fi
		if ((fails >= MAX_RETRIES)); then
			# ip neigh flush to "$ROBOT_IP" nud failed stale reachable 2>/dev/null
			fails=0
		fi
	fi
	sleep "$CHECK_INTERVAL"
done
