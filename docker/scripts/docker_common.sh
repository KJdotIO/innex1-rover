#!/usr/bin/env bash
set -euo pipefail

docker_cmd=()
gpu_runtime_args=()

init_docker_cmd() {
  if docker info >/dev/null 2>&1; then
    docker_cmd=(docker)
    return
  fi

  if command -v sudo >/dev/null 2>&1; then
    docker_cmd=(sudo docker)
    return
  fi

  echo "Docker is not available. Start the Docker daemon or install Docker first." >&2
  exit 1
}

init_gpu_runtime_args() {
  gpu_runtime_args=(
    --gpus all
    -e NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-graphics,compute,utility}"
    -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
  )

  if [[ -d /dev/dri ]]; then
    gpu_runtime_args+=(--device /dev/dri:/dev/dri)

    if [[ -e /dev/dri/card1 ]]; then
      gpu_runtime_args+=(--group-add "$(stat -c '%g' /dev/dri/card1)")
    fi

    if [[ -e /dev/dri/renderD128 ]]; then
      gpu_runtime_args+=(--group-add "$(stat -c '%g' /dev/dri/renderD128)")
    fi
  fi
}
