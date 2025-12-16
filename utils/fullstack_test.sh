#!/usr/bin/env bash
set -euo pipefail

# Full-stack integration test:
# 1. Start the octa_ros dev container (mounts workspace)
# 2. Inside the container: make clean && make build, then ./launch.sh -s
# 3. Verify the driver "standby boot" behavior by toggling URSim offline/online
#    (driver_manager should start/stop the driver launch accordingly)
# 4. Play the recorded bag inside the container
# 5. Wait for result images to be generated
# 6. Tear everything down (URSim + containers)
#
# Usage (from repo root, via Makefile):
#   make test
#
# This script is intended to be invoked by the Makefile and assumes
# it is located under utils/ within the repository.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

COMPOSE_FILE="${1:-docker/docker-compose.yaml}"
URSIM_SCRIPT="${ROOT_DIR}/utils/start_ursim.sh"
URSIM_NAME="ursim"
STACK_CONTAINER_NAME="ps-oce-robot"
DEFAULT_URSIM_IP="192.168.56.101"

echo "[test] Repository root: ${ROOT_DIR}"
echo "[test] Using compose file: ${COMPOSE_FILE}"

if ! command -v docker >/dev/null 2>&1; then
  echo "[test] ERROR: docker is required but not found in PATH" >&2
  exit 1
fi

if [ ! -x "${URSIM_SCRIPT}" ]; then
  echo "[test] ERROR: URSim script not found or not executable: ${URSIM_SCRIPT}" >&2
  exit 1
fi

mkdir -p "${ROOT_DIR}/logs" "${ROOT_DIR}/result"
: > "${ROOT_DIR}/logs/ursim_test.log"

# Start with a clean result directory so we only consider images from this run.
rm -rf "${ROOT_DIR}/result"/*

URSIM_PID=""

stack_proc_running() {
  local pattern="$1"
  docker exec -e PROC_PATTERN="${pattern}" "${STACK_CONTAINER_NAME}" bash -lc '
    set -euo pipefail
    ps -eo args -ww | grep -F "$PROC_PATTERN" | grep -F -v grep >/dev/null 2>&1
  '
}

wait_for_stack_proc() {
  local want_state="$1"
  local pattern="$2"
  local timeout_s="$3"

  local start_ts now_ts elapsed last_print_ts
  start_ts="$(date +%s)"
  last_print_ts="${start_ts}"
  while :; do
    now_ts="$(date +%s)"
    elapsed=$((now_ts - start_ts))
    if [ "${elapsed}" -ge "${timeout_s}" ]; then
      echo "[test] ERROR: Timed out (${timeout_s}s) waiting for process '${pattern}' to be ${want_state}" >&2
      return 1
    fi

    if [ $((now_ts - last_print_ts)) -ge 10 ]; then
      echo "[test] Waiting for '${pattern}' to be ${want_state}... (${elapsed}/${timeout_s}s)"
      last_print_ts="${now_ts}"
    fi

    if stack_proc_running "${pattern}"; then
      if [ "${want_state}" = "running" ]; then
        return 0
      fi
    else
      if [ "${want_state}" = "stopped" ]; then
        return 0
      fi
    fi
    sleep 2
  done
}

wait_for_dashboard_ok() {
  local timeout_s="$1"
  local start_ts now_ts elapsed last_print_ts
  start_ts="$(date +%s)"
  last_print_ts="${start_ts}"

  while :; do
    now_ts="$(date +%s)"
    elapsed=$((now_ts - start_ts))
    if [ "${elapsed}" -ge "${timeout_s}" ]; then
      echo "[test] ERROR: Timed out (${timeout_s}s) waiting for /dashboard_client/get_robot_mode to return success=true" >&2
      return 1
    fi

    if [ $((now_ts - last_print_ts)) -ge 10 ]; then
      echo "[test] Waiting for dashboard get_robot_mode success... (${elapsed}/${timeout_s}s)"
      last_print_ts="${now_ts}"
    fi

    if docker exec "${STACK_CONTAINER_NAME}" bash -lc '
      set -eo pipefail
      cd /workspace/app
      source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
      if [ -f "install/setup.bash" ]; then
        source "install/setup.bash"
      fi
      out="$(timeout 10s ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode "{}" 2>&1 || true)"
      echo "$out" | tr "[:upper:]" "[:lower:]" | grep -Eq "success[[:space:]]*[:=][[:space:]]*true"
    '; then
      return 0
    fi

    sleep 3
  done
}

ursim_start() {
  local log_file="${ROOT_DIR}/logs/ursim_test.log"
  echo "[test] Starting URSim using ${URSIM_SCRIPT}..."
  "${URSIM_SCRIPT}" >>"${log_file}" 2>&1 &
  URSIM_PID=$!
  echo "[test] URSim script PID=${URSIM_PID}"

  echo "[test] Waiting for URSim container '${URSIM_NAME}' to be up..."
  for i in $(seq 1 60); do
    if docker ps --format '{{.Names}}' | grep -qx "${URSIM_NAME}"; then
      echo "[test] URSim container is running."
      break
    fi
    if ! kill -0 "${URSIM_PID}" 2>/dev/null; then
      echo "[test] ERROR: URSim script exited unexpectedly. See logs/ursim_test.log" >&2
      exit 1
    fi
    sleep 1
    if [ "${i}" -eq 60 ]; then
      echo "[test] ERROR: Timed out waiting for URSim container '${URSIM_NAME}'" >&2
      exit 1
    fi
  done

  URSIM_IP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${URSIM_NAME}" 2>/dev/null || true)"
  if [[ -z "${URSIM_IP}" ]]; then
    URSIM_IP="${DEFAULT_URSIM_IP}"
  fi
  export ROBOT_IP="${URSIM_IP}"
  echo "[test] Detected URSim IP: ${URSIM_IP}"
}

ursim_stop() {
  # Stop URSim helper script (which will kill the URSim container)
  if [[ -n "${URSIM_PID}" ]] && kill -0 "${URSIM_PID}" 2>/dev/null; then
    echo "[test] Stopping URSim script (PID=${URSIM_PID})..."
    kill "${URSIM_PID}" >/dev/null 2>&1 || true
    wait "${URSIM_PID}" >/dev/null 2>&1 || true
  fi
  URSIM_PID=""

  # Ensure URSim container is not left running
  if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "${URSIM_NAME}"; then
    echo "[test] Removing URSim container (${URSIM_NAME})..."
    docker rm -f "${URSIM_NAME}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  echo "[test] Cleaning up (exit code=${exit_code})..."
  set +e

  if [ "${exit_code}" -ne 0 ] && [ -n "${STACK_LAUNCH_LOG:-}" ] && [ -f "${STACK_LAUNCH_LOG}" ]; then
    echo "[test] Stack launch log (tail -200): ${STACK_LAUNCH_LOG}"
    tail -n 200 "${STACK_LAUNCH_LOG}" | sed -e 's/^/[stack] /'
  fi

  # Tear down the ROS stack container(s)
  if [ -f "${COMPOSE_FILE}" ]; then
    echo "[test] Stopping docker compose stack (${COMPOSE_FILE})..."
    docker compose -f "${COMPOSE_FILE}" down --remove-orphans >/dev/null 2>&1 || true
  fi

  ursim_stop

  echo "[test] Cleanup complete."
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

# Ensure we start from a clean slate even if a prior run was interrupted.
echo "[test] Pre-cleaning any previous stack/URSim..."
docker compose -f "${COMPOSE_FILE}" down --remove-orphans >/dev/null 2>&1 || true
ursim_stop

# Default expected URSim IP (container uses a static IP).
if [[ -n "${ROBOT_IP:-}" && "${ROBOT_IP}" != "${DEFAULT_URSIM_IP}" ]]; then
  echo "[test] WARNING: ROBOT_IP (${ROBOT_IP}) != URSim IP (${DEFAULT_URSIM_IP}); using URSim IP for test."
fi
export ROBOT_IP="${DEFAULT_URSIM_IP}"
echo "[test] Using ROBOT_IP=${ROBOT_IP} for stack container."

echo "[test] Bringing up stack container '${STACK_CONTAINER_NAME}'..."
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
HOST_UID="${HOST_UID}" HOST_GID="${HOST_GID}" ROBOT_IP="${ROBOT_IP}" \
  docker compose -f "${COMPOSE_FILE}" up -d

echo "[test] Waiting for stack container '${STACK_CONTAINER_NAME}'..."
for i in $(seq 1 60); do
  if docker ps --format '{{.Names}}' | grep -qx "${STACK_CONTAINER_NAME}"; then
    echo "[test] Stack container is running."
    break
  fi
  sleep 2
  if [ "${i}" -eq 60 ]; then
    echo "[test] ERROR: Timed out waiting for stack container '${STACK_CONTAINER_NAME}'" >&2
    exit 1
  fi
done

# Build workspace inside dev container
echo "[test] Building workspace inside container '${STACK_CONTAINER_NAME}'..."
docker exec "${STACK_CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Running: make clean && make build"
  make clean
  make build
'

# Launch the ROS stack in simulation mode inside the container.
echo "[test] Launching stack inside container '${STACK_CONTAINER_NAME}' (./launch.sh -s)..."
STACK_LAUNCH_LOG="${ROOT_DIR}/logs/stack_launch.log"
: > "${STACK_LAUNCH_LOG}"
docker exec \
  -e ROBOT_IP="${ROBOT_IP}" \
  -e PING_TIMEOUT="${PING_TIMEOUT:-1}" \
  -e UR_HEALTH_STARTUP_GRACE_SEC="${UR_HEALTH_STARTUP_GRACE_SEC:-30}" \
  -e UR_HEALTH_STALE_SEC="${UR_HEALTH_STALE_SEC:-15}" \
  -e UR_HEALTH_UNHEALTHY_SEC="${UR_HEALTH_UNHEALTHY_SEC:-15}" \
  -e RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-5}" \
  -e RESTART_COOLDOWN_SEC="${RESTART_COOLDOWN_SEC:-10}" \
  "${STACK_CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Starting ./launch.sh -s (simulation)..."
  ./launch.sh -s
' >"${STACK_LAUNCH_LOG}" 2>&1 &

echo "[test] Waiting for driver_manager process..."
wait_for_stack_proc "running" "driver_manager.py" 60

# Standby boot test: start with URSim offline, then bring it online/offline a few times and
# assert driver launch starts/stops accordingly.
STANDBY_BOOT_CYCLES="${STANDBY_BOOT_CYCLES:-3}"
DRIVER_START_TIMEOUT_S="${DRIVER_START_TIMEOUT_S:-180}"
DRIVER_STOP_TIMEOUT_S="${DRIVER_STOP_TIMEOUT_S:-120}"
DASHBOARD_OK_TIMEOUT_S="${DASHBOARD_OK_TIMEOUT_S:-180}"

echo "[test] Standby boot test: URSim offline->online cycles=${STANDBY_BOOT_CYCLES}"
echo "[test] Ensuring driver is not running while URSim is offline..."
ursim_stop
echo "[test] Waiting for driver launch to stop..."
wait_for_stack_proc "stopped" "ros2 launch octa_ros launch.py" "${DRIVER_STOP_TIMEOUT_S}"

for cycle in $(seq 1 "${STANDBY_BOOT_CYCLES}"); do
  echo "[test] [standby] Cycle ${cycle}/${STANDBY_BOOT_CYCLES}: starting URSim..."
  ursim_start

  echo "[test] [standby] Waiting for driver launch to start..."
  wait_for_stack_proc "running" "ros2 launch octa_ros launch.py" "${DRIVER_START_TIMEOUT_S}"

  echo "[test] [standby] Waiting for dashboard service health..."
  wait_for_dashboard_ok "${DASHBOARD_OK_TIMEOUT_S}"
  echo "[test] [standby] Driver boot OK."

  if [ "${cycle}" -lt "${STANDBY_BOOT_CYCLES}" ]; then
    echo "[test] [standby] Stopping URSim (simulate offline)..."
    ursim_stop
    echo "[test] [standby] Waiting for driver launch to stop..."
    wait_for_stack_proc "stopped" "ros2 launch octa_ros launch.py" "${DRIVER_STOP_TIMEOUT_S}"
  fi
done

echo "[test] Dead-state recovery test: simulate a dead dashboard client and verify manager restarts the stack..."
docker exec "${STACK_CONTAINER_NAME}" bash -lc '
  set -euo pipefail
  pkill -f "ur_robot_driver/[d]ashboard_client" 2>/dev/null || pkill -f "[d]ashboard_client" 2>/dev/null || true
'
echo "[test] Waiting for driver launch to stop (health-triggered restart)..."
wait_for_stack_proc "stopped" "ros2 launch octa_ros launch.py" "${DRIVER_STOP_TIMEOUT_S}"
echo "[test] Waiting for driver launch to start again..."
wait_for_stack_proc "running" "ros2 launch octa_ros launch.py" "${DRIVER_START_TIMEOUT_S}"
echo "[test] Waiting for dashboard service health after restart..."
wait_for_dashboard_ok "${DASHBOARD_OK_TIMEOUT_S}"
echo "[test] Dead-state recovery OK."

# Give the ROS stack a bit of time to come up internally before playing the bag.
STACK_STARTUP_DELAY_S="${STACK_STARTUP_DELAY_S:-30}"
echo "[test] Waiting ${STACK_STARTUP_DELAY_S}s for ROS 2 stack to initialize..."
sleep "${STACK_STARTUP_DELAY_S}"

echo "[test] Starting bag playback inside container '${STACK_CONTAINER_NAME}'..."
docker exec "${STACK_CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Sourcing ROS and workspace..."
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
  fi

  BAG_DIR="/workspace/app/bags/fullscan"
  QOS_YAML="/workspace/app/config/bag_qos.yaml"

  echo "[container] BAG_DIR=${BAG_DIR}"
  echo "[container] QOS_YAML=${QOS_YAML}"

  if [ ! -d "${BAG_DIR}" ]; then
    echo "[container] ERROR: Bag directory not found: ${BAG_DIR}" >&2
    exit 1
  fi

  if [ ! -f "${QOS_YAML}" ]; then
    echo "[container] WARNING: QoS override file not found: ${QOS_YAML}; running without overrides."
    QOS_YAML=""
  fi

  CMD=(ros2 bag play "${BAG_DIR}" --disable-loan-message)
  if [ -n "${QOS_YAML}" ]; then
    CMD+=(--qos-profile-overrides-path "${QOS_YAML}")
  fi

  echo "[container] Running: ${CMD[*]}"
  timeout "${BAG_TIMEOUT_S:-900}s" "${CMD[@]}"
'

echo "[test] Bag playback finished; checking for result images..."

RESULT_ROOT="${ROOT_DIR}/result"
start_ts="$(date +%s)"
timeout_s="${RESULT_TIMEOUT_S:-900}"
found=0

while :; do
  now_ts="$(date +%s)"
  elapsed=$((now_ts - start_ts))
  if [ "${elapsed}" -ge "${timeout_s}" ]; then
    echo "[test] ERROR: Timed out (${timeout_s}s) waiting for result images." >&2
    break
  fi

  if [ -d "${RESULT_ROOT}" ]; then
    # Find newest session directory under result/
    latest="$(find "${RESULT_ROOT}" -mindepth 1 -maxdepth 1 -type d -printf "%T@ %p\n" 2>/dev/null \
      | sort -nr | head -n1 | cut -d" " -f2-)"
    if [ -n "${latest}" ]; then
      raw_count="$(find "${latest}" -maxdepth 1 -type f -name "raw_image*.jpg" | wc -l || echo 0)"
      det_count="$(find "${latest}" -maxdepth 1 -type f -name "detected_image*.jpg" | wc -l || echo 0)"
      if [ "${raw_count}" -ge 6 ] && [ "${det_count}" -ge 6 ]; then
        echo "[test] Found ${raw_count} raw and ${det_count} detected images in ${latest}"
        found=1
        break
      fi
    fi
  fi

  sleep 5
done

if [ "${found}" -ne 1 ]; then
  echo "[test] ERROR: Result images not found or insufficient (need >=6 raw and >=6 detected)." >&2
  exit 1
fi

echo "[test] Full-stack test completed successfully."
