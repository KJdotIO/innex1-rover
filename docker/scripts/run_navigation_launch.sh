#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./docker_common.sh
source "${SCRIPT_DIR}/docker_common.sh"

IMAGE_NAME="${IMAGE_NAME:-innex1/sim-dev:local}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER_NAME="${CONTAINER_NAME:-innex1-navigation}"
DETACH="${DETACH:-false}"

init_docker_cmd
init_gpu_runtime_args

"${docker_cmd[@]}" rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker_args=(run --rm --name "${CONTAINER_NAME}")
if [[ "${DETACH}" == "true" ]]; then
  docker_args+=(-d)
else
  docker_args+=(-it)
fi

"${docker_cmd[@]}" "${docker_args[@]}" \
  --net=host \
  --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  "${gpu_runtime_args[@]}" \
  -v "$(pwd):/workspaces/innex1-rover" \
  -w /workspaces/innex1-rover \
  "${IMAGE_NAME}" \
  bash -lc "if [[ ! -f install/setup.bash ]]; then echo 'Workspace is not built yet. Run ./docker/scripts/run_sim_dev.sh and colcon build --symlink-install first.' >&2; exit 1; fi; ros2 launch lunabot_bringup navigation.launch.py"
