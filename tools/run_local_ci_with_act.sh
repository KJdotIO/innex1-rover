#!/usr/bin/env bash
set -euo pipefail

job_id="${1:-preflight}"
base_ref="${2:-main}"
workflow_path="${3:-.github/workflows/ci.yml}"
shift $(( $# >= 3 ? 3 : $# ))

if ! command -v act >/dev/null 2>&1; then
  echo "act is not installed."
  echo "Install it with: brew install act"
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "Docker is not available. Start OrbStack first."
  exit 1
fi

if ! git rev-parse --verify "origin/${base_ref}" >/dev/null 2>&1; then
  echo "Remote branch origin/${base_ref} was not found."
  echo "Fetch it first, or pass a different base ref."
  exit 1
fi

head_sha="$(git rev-parse HEAD)"
base_sha="$(git merge-base HEAD "origin/${base_ref}")"
event_file="$(mktemp "${TMPDIR:-/tmp}/act-pr-event.XXXXXX")"

cleanup() {
  rm -f "$event_file"
}
trap cleanup EXIT

cat >"$event_file" <<EOF
{
  "pull_request": {
    "base": {
      "ref": "${base_ref}",
      "sha": "${base_sha}"
    },
    "head": {
      "sha": "${head_sha}"
    }
  }
}
EOF

echo "Running job '${job_id}' from '${workflow_path}' against base '${base_ref}'."
echo "Using event payload: ${event_file}"

exec act pull_request \
  --job "${job_id}" \
  --workflows "${workflow_path}" \
  --eventpath "${event_file}" \
  "$@"
