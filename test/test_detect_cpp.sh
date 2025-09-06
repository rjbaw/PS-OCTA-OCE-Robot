#!/usr/bin/env bash
set -euo pipefail

# Usage: test_detect_cpp.sh [INPUT_PATH] [OUTPUT_DIR]
# - INPUT_PATH: directory or single image. Defaults to test/data/raw
# - OUTPUT_DIR: where to write results. Defaults to test/data/result_cpp

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

INPUT_PATH="${1:-${SCRIPT_DIR}/data/raw}"
OUT_DIR="${2:-${SCRIPT_DIR}/data/result_cpp}"

# Source ROS 2 and local overlay
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${SCRIPT_DIR}/../install/setup.bash"
set -u

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

timeout "${TIMEOUT_S:-900}s" ros2 run octa_ros test_detect "${INPUT_PATH}" "${OUT_DIR}"
