#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./docker_common.sh
source "${SCRIPT_DIR}/docker_common.sh"

IMAGE_NAME="${IMAGE_NAME:-innex1/sim-dev:local}"
PLATFORM="${PLATFORM:-linux/amd64}"
HOST_UID="${SUDO_UID:-$(python3 -c 'import os; print(os.stat(".").st_uid)')}"
HOST_GID="${SUDO_GID:-$(python3 -c 'import os; print(os.stat(".").st_gid)')}"
USE_BUILDX="${USE_BUILDX:-false}"

init_docker_cmd

if [[ "${USE_BUILDX}" == "true" ]]; then
  "${docker_cmd[@]}" buildx build \
    --load \
    --platform "${PLATFORM}" \
    --target sim-dev \
    --build-arg UID="${HOST_UID}" \
    --build-arg GID="${HOST_GID}" \
    -f docker/Dockerfile \
    -t "${IMAGE_NAME}" \
    .
else
  "${docker_cmd[@]}" build \
    --platform "${PLATFORM}" \
    --target sim-dev \
    --build-arg UID="${HOST_UID}" \
    --build-arg GID="${HOST_GID}" \
    -f docker/Dockerfile \
    -t "${IMAGE_NAME}" \
    .
fi
