#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${SCRIPT_DIR}/../../../install/setup.bash" ]]; then
  ROOT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
  WORKSPACE_SETUP="${ROOT_DIR}/install/setup.bash"
elif [[ -f "${SCRIPT_DIR}/../../../setup.bash" ]]; then
  ROOT_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
  WORKSPACE_SETUP="${SCRIPT_DIR}/../../../setup.bash"
else
  echo "Could not locate workspace setup.bash from ${SCRIPT_DIR}"
  exit 1
fi

if [[ -f "${SCRIPT_DIR}/check_sim_topics.py" ]]; then
  HELPER_DIR="${SCRIPT_DIR}"
else
  HELPER_DIR="${ROOT_DIR}/src/lunabot_simulation/scripts"
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Missing ${WORKSPACE_SETUP}"
  echo "Build the workspace first: colcon build --symlink-install"
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "Missing /opt/ros/humble/setup.bash"
  echo "Install ROS 2 Humble and source its setup script first"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_SETUP}"

LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"

echo "Starting Gazebo Classic simulation..."
ros2 launch lunabot_simulation moon_yard.launch.py >"${LOG_DIR}/classic-smoke-launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" || true
    wait "${LAUNCH_PID}" || true
  fi
}
trap cleanup EXIT

sleep 6

echo "Checking topic discovery..."
python3 "${HELPER_DIR}/check_sim_topics.py" --timeout 60

declare -a TOPICS=(
  "/clock"
  "/odom"
  "/joint_states"
  "/imu/data_raw"
  "/scan"
  "/camera_front/image"
  "/camera_front/camera_info"
  "/camera_front/depth_image"
  "/camera_front/points"
  "/camera/image_raw"
  "/camera/camera_info"
)

echo "Checking first message on each required topic..."
for topic in "${TOPICS[@]}"; do
  echo "- ${topic}"
  timeout 20 ros2 topic echo --once "${topic}" >/dev/null
done

echo "Checking cmd_vel to odom motion response..."
python3 "${HELPER_DIR}/check_motion_response.py" --timeout 30 --drive-seconds 4 --linear 0.25 --min-distance 0.03

echo "Smoke test passed"
