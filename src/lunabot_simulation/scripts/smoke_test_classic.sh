#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Missing ${ROOT_DIR}/install/setup.bash"
  echo "Build the workspace first: colcon build --symlink-install"
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "Missing /opt/ros/humble/setup.bash"
  echo "Install ROS 2 Humble and source its setup script first"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"

echo "Starting Gazebo Classic simulation..."
ros2 launch lunabot_simulation moon_yard.launch.py >"${ROOT_DIR}/logs/classic-smoke-launch.log" 2>&1 &
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
python3 "${ROOT_DIR}/src/lunabot_simulation/scripts/check_sim_topics.py" --timeout 60

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
python3 "${ROOT_DIR}/src/lunabot_simulation/scripts/check_motion_response.py" --timeout 30 --drive-seconds 4 --linear 0.25 --min-distance 0.03

echo "Smoke test passed"
