#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash || true
source install/setup.bash 2>/dev/null || true
source /workspace/app/setup.bash 2>/dev/null || true

pkill -f 'ros2|octa_ros|coordinator_nod|focus_node|move_node|reset_node|freedrive_node|joint_state_pub|reconnect_clien|controller_manager|move_group|ros2_control_node|dashboard_client|urscript_interface|robot_state_publisher|rviz2' 2>/dev/null || true
sleep 0.5

help() {
	cat <<EOF
Launch OCTA/OCE ROS manager

Usage: ./launch.sh [-h|-d|-s]

Environment variables:
  ROBOT_IP           Robot IP (default 192.168.0.10)
  HOST_IP            Host IP for reverse connection (auto-detected if unset)
  LOG_DIR            Logs directory (default ./logs)
  PRUNE_LOGS_DAYS    Days before log pruning (default 7)
  PING_TIMEOUT       Ping timeout seconds for reachability (default 3)

Behavior:
  - Starts the driver manager which launches/stops the ROS driver based on:
      IP offline       -> stop
      IP online + LV=false -> stop
      IP online + LV=true or no message -> start
  - RCUTILS logs are written under LOG_DIR.

Options:
  -d  Debug: run launch.py directly (bypass manager)
  -s  Simulation: if ROBOT_IP is unset, default to 192.168.56.101
EOF
}

debug=false
sim=false
while getopts ":hds" opt; do
	case "$opt" in
	h)
		help
		exit 0
		;;
	d) debug=true ;;
	s) sim=true ;;
	\?)
		echo "Invalid option: -$OPTARG" >&2
		exit 2
		;;
	esac
done

LOG_DIR="${LOG_DIR:-$PWD/logs}"
PRUNE_LOGS_DAYS="${PRUNE_LOGS_DAYS:-7}"
mkdir -p "$LOG_DIR"
find "$LOG_DIR" -mindepth 1 -mtime +"$PRUNE_LOGS_DAYS" -exec rm -rf {} + 2>/dev/null || true

if [[ -z "${ROBOT_IP:-}" ]]; then
  if $sim; then
    ROBOT_IP="192.168.56.101"
  else
    ROBOT_IP="192.168.0.10"
  fi
fi

# Allow overriding robot type via env
UR_TYPE="${UR_TYPE:-ur3e}"
if [[ -z "${HOST_IP:-}" ]]; then
	detected_ip=$(ip route get "$ROBOT_IP" 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')
	HOST_IP=${detected_ip:-192.168.0.2}
fi

export RCUTILS_LOGGING_DIRECTORY="$LOG_DIR"
export ROS_LOG_DIR="$LOG_DIR"
export ROBOT_IP
export HOST_IP
echo "[INFO] Logs dir: $LOG_DIR"
if $debug; then
	echo "[INFO] Debug mode: launching driver directly (bypassing manager)"
	echo "[INFO] ROBOT_IP=$ROBOT_IP HOST_IP=$HOST_IP"
	cd /workspace/app 2>/dev/null || true
	exec ros2 launch octa_ros launch.py ur_type:=ur3e robot_ip:=$ROBOT_IP headless_mode:=true reverse_ip:=$HOST_IP
else
  echo "[INFO] Manager starting (ROBOT_IP=$ROBOT_IP HOST_IP=$HOST_IP)"
  exec ros2 run octa_ros driver_manager.py \
    --robot-ip "$ROBOT_IP" \
    --host-ip "$HOST_IP" \
    --ur-type "$UR_TYPE" \
    --headless \
    --ping-timeout "${PING_TIMEOUT:-3}"
fi
