#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./docker_common.sh
source "${SCRIPT_DIR}/docker_common.sh"

IMAGE_NAME="${IMAGE_NAME:-innex1/sim-dev:local}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER_NAME="${CONTAINER_NAME:-innex1-sim-dev}"

init_docker_cmd
init_gpu_runtime_args

"${docker_cmd[@]}" run --rm -it \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  "${gpu_runtime_args[@]}" \
  -v "$(pwd):/workspaces/innex1-rover" \
  -w /workspaces/innex1-rover \
  "${IMAGE_NAME}" \
  bash
