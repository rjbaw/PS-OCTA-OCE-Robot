#!/usr/bin/env bash
set -euo pipefail

# Full-stack integration test:
# 1. Start URSim via utils/start_ursim.sh
# 2. Start the octa_ros dev container (mounts workspace)
# 3. Inside the container: make clean && make build, then ./launch.sh -s
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

# Start with a clean result directory so we only consider images from this run.
rm -rf "${ROOT_DIR}/result"/*

URSIM_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  echo "[test] Cleaning up (exit code=${exit_code})..."
  set +e

  # Tear down the ROS stack container(s)
  if [ -f "${COMPOSE_FILE}" ]; then
    echo "[test] Stopping docker compose stack (${COMPOSE_FILE})..."
    docker compose -f "${COMPOSE_FILE}" down --remove-orphans >/dev/null 2>&1 || true
  fi

  # Stop URSim helper script (which will kill the URSim container)
  if [[ -n "${URSIM_PID}" ]] && kill -0 "${URSIM_PID}" 2>/dev/null; then
    echo "[test] Stopping URSim script (PID=${URSIM_PID})..."
    kill "${URSIM_PID}" >/dev/null 2>&1 || true
    wait "${URSIM_PID}" >/dev/null 2>&1 || true
  fi

  # Ensure URSim container is not left running
  if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "${URSIM_NAME}"; then
    echo "[test] Removing URSim container (${URSIM_NAME})..."
    docker rm -f "${URSIM_NAME}" >/dev/null 2>&1 || true
  fi

  echo "[test] Cleanup complete."
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

echo "[test] Starting URSim using ${URSIM_SCRIPT}..."
"${URSIM_SCRIPT}" >"${ROOT_DIR}/logs/ursim_test.log" 2>&1 &
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

echo "[test] Detected URSim IP: ${URSIM_IP}"

if [[ -n "${ROBOT_IP:-}" && "${ROBOT_IP}" != "${URSIM_IP}" ]]; then
  echo "[test] WARNING: ROBOT_IP (${ROBOT_IP}) != URSim IP (${URSIM_IP}); using URSim IP for test."
fi

export ROBOT_IP="${URSIM_IP}"
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
docker exec -e ROBOT_IP="${ROBOT_IP}" "${STACK_CONTAINER_NAME}" bash -lc '
  set -eo pipefail
  cd /workspace/app
  echo "[container] Starting ./launch.sh -s (simulation)..."
  ./launch.sh -s
' &

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

  CMD=(ros2 bag play "${BAG_DIR}")
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
