#!/usr/bin/env bash
set -euo pipefail

# Full-stack integration test:
# 1. Start the octa_ros dev container (mounts workspace)
# 2. Inside the container: make clean && make build, then ./launch.sh -s
# 3. Verify the driver "standby boot" behavior by toggling URSim offline/online
#    (driver_manager should start/stop the driver launch accordingly)
# 4. Play the recorded Fullscan and motion bags inside the container
# 5. Verify the recorded actions and wait for result images
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
URSIM_LOG="${ROOT_DIR}/logs/ursim_test.log"
STACK_LAUNCH_LOG="${ROOT_DIR}/logs/stack_launch.log"
STACK_EXEC_PID=""

STANDBY_BOOT_CYCLES="${STANDBY_BOOT_CYCLES:-3}"
DRIVER_START_TIMEOUT_S="${DRIVER_START_TIMEOUT_S:-180}"
DRIVER_STOP_TIMEOUT_S="${DRIVER_STOP_TIMEOUT_S:-120}"
DASHBOARD_OK_TIMEOUT_S="${DASHBOARD_OK_TIMEOUT_S:-180}"
STACK_STARTUP_DELAY_S="${STACK_STARTUP_DELAY_S:-30}"
BAG_TIMEOUT_S="${BAG_TIMEOUT_S:-900}"
ACTION_TIMEOUT_S="${ACTION_TIMEOUT_S:-120}"
RESULT_TIMEOUT_S="${RESULT_TIMEOUT_S:-900}"
RESULT_MIN_IMAGES="${RESULT_MIN_IMAGES:-6}"

PING_TIMEOUT="${PING_TIMEOUT:-1}"
UR_HEALTH_STARTUP_GRACE_SEC="${UR_HEALTH_STARTUP_GRACE_SEC:-30}"
UR_HEALTH_STALE_SEC="${UR_HEALTH_STALE_SEC:-15}"
UR_HEALTH_UNHEALTHY_SEC="${UR_HEALTH_UNHEALTHY_SEC:-15}"
RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-5}"
RESTART_COOLDOWN_SEC="${RESTART_COOLDOWN_SEC:-10}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"

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
: > "${URSIM_LOG}"
: > "${STACK_LAUNCH_LOG}"

# Start with a clean result directory so we only consider images from this run.
rm -rf "${ROOT_DIR}/result"/*

URSIM_PID=""

docker_container_running() {
  local name="$1"
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${name}"
}

wait_for_docker_container() {
  local name="$1"
  local timeout_s="$2"
  local watched_pid="${3:-}"

  local start_ts now_ts elapsed last_print_ts
  start_ts="$(date +%s)"
  last_print_ts="${start_ts}"
  while :; do
    now_ts="$(date +%s)"
    elapsed=$((now_ts - start_ts))
    if [ "${elapsed}" -ge "${timeout_s}" ]; then
      echo "[test] ERROR: Timed out (${timeout_s}s) waiting for container '${name}' to be running" >&2
      return 1
    fi

    if docker_container_running "${name}"; then
      return 0
    fi

    if [ -n "${watched_pid}" ] && ! kill -0 "${watched_pid}" 2>/dev/null; then
      echo "[test] ERROR: Helper process exited while waiting for container '${name}' (pid=${watched_pid})." >&2
      return 1
    fi

    if [ $((now_ts - last_print_ts)) -ge 10 ]; then
      echo "[test] Waiting for container '${name}'... (${elapsed}/${timeout_s}s)"
      last_print_ts="${now_ts}"
    fi

    sleep 1
  done
}

stack_exec() {
  local cmd="$1"
  docker exec "${STACK_CONTAINER_NAME}" bash -lc "${cmd}"
}

stack_exec_opts() {
  local cmd="$1"
  shift
  docker exec "$@" "${STACK_CONTAINER_NAME}" bash -lc "${cmd}"
}

stack_proc_pid() {
  local pattern="$1"
  docker exec -e PROC_PATTERN="${pattern}" "${STACK_CONTAINER_NAME}" bash -lc '
    set -euo pipefail
    ps -eo pid=,args= -ww | grep -F "$PROC_PATTERN" | grep -F -v grep | head -n 1 | awk "{print \$1}"
  ' 2>/dev/null || true
}

stack_proc_running() {
  local pattern="$1"
  [ -n "$(stack_proc_pid "${pattern}")" ]
}

stack_pid_exists() {
  local pid="$1"
  docker exec -e PID="${pid}" "${STACK_CONTAINER_NAME}" bash -lc '
    set -euo pipefail
    ps -p "$PID" >/dev/null 2>&1
  ' 2>/dev/null
}

wait_for_proc_restart() {
  local pattern="$1"
  local old_pid="$2"
  local timeout_s="$3"

  local start_ts now_ts elapsed last_print_ts
  start_ts="$(date +%s)"
  last_print_ts="${start_ts}"
  while :; do
    now_ts="$(date +%s)"
    elapsed=$((now_ts - start_ts))
    if [ "${elapsed}" -ge "${timeout_s}" ]; then
      echo "[test] ERROR: Timed out (${timeout_s}s) waiting for process '${pattern}' to restart (old pid=${old_pid})" >&2
      return 1
    fi

    if [ $((now_ts - last_print_ts)) -ge 10 ]; then
      echo "[test] Waiting for '${pattern}' to restart... (${elapsed}/${timeout_s}s)"
      last_print_ts="${now_ts}"
    fi

    current_pid="$(stack_proc_pid "${pattern}" || true)"
    if [ -n "${current_pid}" ] && [ "${current_pid}" != "${old_pid}" ]; then
      if ! stack_pid_exists "${old_pid}"; then
        echo "[test] Detected restart: pid ${old_pid} -> ${current_pid}"
        return 0
      fi
    fi

    sleep 2
  done
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

    if stack_exec '
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
  echo "[test] Starting URSim using ${URSIM_SCRIPT}..."
  "${URSIM_SCRIPT}" >>"${URSIM_LOG}" 2>&1 &
  URSIM_PID=$!
  echo "[test] URSim script PID=${URSIM_PID}"

  echo "[test] Waiting for URSim container '${URSIM_NAME}' to be up..."
  if ! wait_for_docker_container "${URSIM_NAME}" 60 "${URSIM_PID}"; then
    echo "[test] ERROR: URSim failed to start. See ${URSIM_LOG}" >&2
    exit 1
  fi
  echo "[test] URSim container is running."

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

  if [ -n "${STACK_EXEC_PID}" ] && kill -0 "${STACK_EXEC_PID}" 2>/dev/null; then
    kill "${STACK_EXEC_PID}" >/dev/null 2>&1 || true
    wait "${STACK_EXEC_PID}" >/dev/null 2>&1 || true
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
if ! wait_for_docker_container "${STACK_CONTAINER_NAME}" 120; then
  exit 1
fi
echo "[test] Stack container is running."

# Build workspace inside dev container
echo "[test] Building workspace inside container '${STACK_CONTAINER_NAME}'..."
stack_exec '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Running: make clean && make build"
  make clean
  make build
'

# Launch the ROS stack in simulation mode inside the container.
echo "[test] Launching stack inside container '${STACK_CONTAINER_NAME}' (./launch.sh -s)..."
: > "${STACK_LAUNCH_LOG}"
stack_env_args=(
  -e "ROBOT_IP=${ROBOT_IP}"
  -e "PING_TIMEOUT=${PING_TIMEOUT}"
  -e "UR_HEALTH_STARTUP_GRACE_SEC=${UR_HEALTH_STARTUP_GRACE_SEC}"
  -e "UR_HEALTH_STALE_SEC=${UR_HEALTH_STALE_SEC}"
  -e "UR_HEALTH_UNHEALTHY_SEC=${UR_HEALTH_UNHEALTHY_SEC}"
  -e "RESTART_DELAY_SEC=${RESTART_DELAY_SEC}"
  -e "RESTART_COOLDOWN_SEC=${RESTART_COOLDOWN_SEC}"
  -e "LAUNCH_RVIZ=${LAUNCH_RVIZ}"
)
docker exec "${stack_env_args[@]}" "${STACK_CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Starting ./launch.sh -s (simulation)..."
  ./launch.sh -s
' >"${STACK_LAUNCH_LOG}" 2>&1 &
STACK_EXEC_PID=$!

echo "[test] Waiting for driver_manager process..."
wait_for_stack_proc "running" "driver_manager.py" 60

# Standby boot test: start with URSim offline, then bring it online/offline a few times and
# assert driver launch starts/stops accordingly.
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
old_launch_pid="$(stack_proc_pid "ros2 launch octa_ros launch.py" || true)"
if [ -z "${old_launch_pid}" ]; then
  echo "[test] ERROR: Could not determine current 'ros2 launch octa_ros launch.py' PID before dead-state test" >&2
  exit 1
fi
stack_exec '
  set -euo pipefail
  pkill -f "ur_robot_driver/[d]ashboard_client" 2>/dev/null || pkill -f "[d]ashboard_client" 2>/dev/null || true
'
echo "[test] Waiting for driver launch to restart (health-triggered)..."
wait_for_proc_restart "ros2 launch octa_ros launch.py" "${old_launch_pid}" "${DRIVER_START_TIMEOUT_S}"
echo "[test] Waiting for dashboard service health after restart..."
wait_for_dashboard_ok "${DASHBOARD_OK_TIMEOUT_S}"
echo "[test] Dead-state recovery OK."

# Give the ROS stack a bit of time to come up internally before playing the bag.
echo "[test] Waiting ${STACK_STARTUP_DELAY_S}s for ROS 2 stack to initialize..."
sleep "${STACK_STARTUP_DELAY_S}"

log_count() {
  local pattern="$1"
  grep -F -c "${pattern}" "${STACK_LAUNCH_LOG}" || true
}

wait_for_log_increment() {
  local description="$1"
  local pattern="$2"
  local baseline="$3"
  local increment="$4"
  local timeout_s="${5:-${ACTION_TIMEOUT_S}}"
  local target start_ts now_ts count
  target=$((baseline + increment))
  start_ts="$(date +%s)"

  while :; do
    count="$(log_count "${pattern}")"
    if [ "${count}" -ge "${target}" ]; then
      echo "[test] Verified ${description}: +$((count - baseline))"
      return 0
    fi
    now_ts="$(date +%s)"
    if [ $((now_ts - start_ts)) -ge "${timeout_s}" ]; then
      echo "[test] ERROR: Timed out waiting for ${description}; expected +${increment}, found +$((count - baseline))." >&2
      return 1
    fi
    sleep 1
  done
}

assert_log_delta() {
  local description="$1"
  local pattern="$2"
  local baseline="$3"
  local expected="$4"
  local count delta
  count="$(log_count "${pattern}")"
  delta=$((count - baseline))
  if [ "${delta}" -ne "${expected}" ]; then
    echo "[test] ERROR: Expected ${description} exactly ${expected} time(s), found ${delta}." >&2
    return 1
  fi
  echo "[test] Verified ${description}: +${delta}"
}

replay_bag() {
  local bag_name="$1"
  echo "[test] Replaying ${bag_name} inside container '${STACK_CONTAINER_NAME}'..."
  stack_exec_opts '
    set -eo pipefail
    cd /workspace/app
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    if [ -f "install/setup.bash" ]; then
      source "install/setup.bash"
    fi

    BAG_DIR="/workspace/app/bags/${BAG_NAME}"
    QOS_YAML="/workspace/app/config/bag_qos.yaml"
    if [ ! -d "${BAG_DIR}" ]; then
      echo "[container] ERROR: Bag directory not found: ${BAG_DIR}" >&2
      exit 1
    fi
    if [ ! -f "${QOS_YAML}" ]; then
      echo "[container] WARNING: QoS override file not found: ${QOS_YAML}; running without overrides."
      QOS_YAML=""
    fi

    CMD=(ros2 bag play "${BAG_DIR}" --disable-loan-message)
    if [ "${BAG_NAME}" != "fullscan" ]; then
      CMD+=(--topics /labview_data)
    fi
    if [ -n "${QOS_YAML}" ]; then
      CMD+=(--qos-profile-overrides-path "${QOS_YAML}")
    fi

    echo "[container] Running: ${CMD[*]}"
    timeout "${BAG_TIMEOUT_S}s" "${CMD[@]}"
  ' -e "BAG_NAME=${bag_name}" -e "BAG_TIMEOUT_S=${BAG_TIMEOUT_S}"
}

verify_rotate_z() {
  local description="$1"
  local next_before previous_before move_before
  local next_delta previous_delta expected_moves

  next_before="$(log_count "[Action] Next:")"
  previous_before="$(log_count "[Action] Previous:")"
  move_before="$(log_count "Move SUCCEEDED")"
  replay_bag rotate_z
  wait_for_log_increment "${description} Next request" "[Action] Next:" "${next_before}" 1
  wait_for_log_increment "${description} Previous request" "[Action] Previous:" "${previous_before}" 1
  # Bag playback can exit just before the last DDS sample reaches the
  # coordinator log. Give the optional second Previous edge time to arrive.
  sleep 2
  next_delta=$(( $(log_count "[Action] Next:") - next_before ))
  previous_delta=$(( $(log_count "[Action] Previous:") - previous_before ))
  if [ "${next_delta}" -ne 1 ] ||
     [ "${previous_delta}" -lt 1 ] || [ "${previous_delta}" -gt 2 ]; then
    echo "[test] ERROR: ${description} dispatched ${next_delta} Next and ${previous_delta} Previous request(s); expected 1 Next and 1-2 Previous." >&2
    return 1
  fi
  expected_moves=$((next_delta + previous_delta))
  echo "[test] Verified ${description} requests: Next=${next_delta}, Previous=${previous_delta}"
  wait_for_log_increment "${description} successful moves" "Move SUCCEEDED" "${move_before}" "${expected_moves}"
  assert_log_delta "${description} successful moves" "Move SUCCEEDED" "${move_before}" "${expected_moves}"
}

echo "[test] Starting bag replays: fullscan, rotate_z x2, home, reset, post-reset rotate_z"
startup_freedrive_wait_start="$(date +%s)"
while :; do
  coordinator_init_line="$(grep -n -F "Coordinator Node Initialized." "${STACK_LAUNCH_LOG}" | tail -n 1 | cut -d: -f1 || true)"
  freedrive_success_line="$(grep -n -F "Freedrive SUCCESS" "${STACK_LAUNCH_LOG}" | tail -n 1 | cut -d: -f1 || true)"
  if [ -n "${coordinator_init_line}" ] && [ -n "${freedrive_success_line}" ] &&
     [ "${freedrive_success_line}" -gt "${coordinator_init_line}" ]; then
    echo "[test] Verified startup Freedrive OFF confirmation for the current coordinator."
    break
  fi
  if [ "$(( $(date +%s) - startup_freedrive_wait_start ))" -ge "${ACTION_TIMEOUT_S}" ]; then
    echo "[test] ERROR: Timed out waiting for startup Freedrive OFF confirmation from the current coordinator." >&2
    exit 1
  fi
  sleep 1
done

fullscan_request_before="$(log_count "[Fullscan] REQUEST_ACCEPTED")"
fullscan_complete_before="$(log_count "[Fullscan] COMPLETE:")"
fullscan_cancel_before="$(log_count "[Fullscan] CANCEL:")"
fullscan_abort_before="$(log_count "[Fullscan] ABORT:")"
focus_scan3d_timeout_before="$(log_count "activate_3d_scan not responding...")"
focus_abort_before="$(log_count "Focus action ABORTED")"
step_zero_cancel_before="$(log_count "[Fullscan] CANCEL: completed_steps=0")"
fullscan_known_step_zero_cancel=0
replay_bag fullscan
wait_for_log_increment "Fullscan request" "[Fullscan] REQUEST_ACCEPTED" "${fullscan_request_before}" 1
assert_log_delta "Fullscan request" "[Fullscan] REQUEST_ACCEPTED" "${fullscan_request_before}" 1

fullscan_wait_start="$(date +%s)"
while :; do
  if [ "$(log_count "[Fullscan] COMPLETE:")" -gt "${fullscan_complete_before}" ]; then
    echo "[test] Fullscan replay completed."
    break
  fi
  if [ "$(log_count "[Fullscan] ABORT:")" -gt "${fullscan_abort_before}" ]; then
    echo "[test] ERROR: Fullscan aborted before producing a valid recipe." >&2
    exit 1
  fi
  if [ "$(log_count "[Fullscan] CANCEL:")" -gt "${fullscan_cancel_before}" ]; then
    if [ "$(log_count "activate_3d_scan not responding...")" -gt "${focus_scan3d_timeout_before}" ] &&
       [ "$(log_count "Focus action ABORTED")" -gt "${focus_abort_before}" ] &&
       [ "$(log_count "[Fullscan] CANCEL: completed_steps=0")" -gt "${step_zero_cancel_before}" ]; then
      echo "[test] WARNING: Fullscan reached the recording's known missing Focus Scan3D response and canceled at step zero."
      fullscan_known_step_zero_cancel=1
      break
    fi
    echo "[test] ERROR: Fullscan canceled for an unexpected reason." >&2
    exit 1
  fi
  if [ $(( $(date +%s) - fullscan_wait_start )) -ge "${ACTION_TIMEOUT_S}" ]; then
    echo "[test] ERROR: Timed out waiting for Fullscan to settle." >&2
    exit 1
  fi
  sleep 1
done

for rotate_run in 1 2; do
  verify_rotate_z "Rotate-Z ${rotate_run}"
done

home_before="$(log_count "[Action] Home:")"
move_before="$(log_count "Move SUCCEEDED")"
replay_bag home
wait_for_log_increment "Home request" "[Action] Home:" "${home_before}" 1
assert_log_delta "Home request" "[Action] Home:" "${home_before}" 1
wait_for_log_increment "Home successful move" "Move SUCCEEDED" "${move_before}" 1
assert_log_delta "Home successful move" "Move SUCCEEDED" "${move_before}" 1

reset_before="$(log_count "Reset SUCCESS")"
freedrive_before="$(log_count "Freedrive SUCCESS")"
replay_bag reset
wait_for_log_increment "Reset success" "Reset SUCCESS" "${reset_before}" 1
assert_log_delta "Reset success" "Reset SUCCESS" "${reset_before}" 1
wait_for_log_increment "post-Reset controller restoration" "Freedrive SUCCESS" "${freedrive_before}" 1
assert_log_delta "post-Reset controller restoration" "Freedrive SUCCESS" "${freedrive_before}" 1
verify_rotate_z "Post-Reset Rotate-Z"

if [ "${fullscan_known_step_zero_cancel}" -eq 1 ]; then
  echo "[test] Skipping result-image verification: the recorded Fullscan cannot reach image capture without its missing Scan3D response."
else
  echo "[test] Checking for result images..."

  RESULT_ROOT="${ROOT_DIR}/result"
  start_ts="$(date +%s)"
  timeout_s="${RESULT_TIMEOUT_S}"
  min_images="${RESULT_MIN_IMAGES}"
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
        if [ "${raw_count}" -ge "${min_images}" ] && [ "${det_count}" -ge "${min_images}" ]; then
          echo "[test] Found ${raw_count} raw and ${det_count} detected images in ${latest}"
          found=1
          break
        fi
      fi
    fi

    sleep 5
  done

  if [ "${found}" -ne 1 ]; then
    echo "[test] ERROR: Result images not found or insufficient (need >=${min_images} raw and >=${min_images} detected)." >&2
    exit 1
  fi
fi

echo "[test] Full-stack test completed successfully."
