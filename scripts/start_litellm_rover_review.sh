#!/usr/bin/env bash
set -euo pipefail

export VIBEPROXY_API_KEY="${VIBEPROXY_API_KEY:-sk-vibeproxy-local}"
export LITELLM_MASTER_KEY="${LITELLM_MASTER_KEY:-sk-local-rover-review}"

uvx --from 'litellm[proxy]' litellm \
  --config .github/codex/litellm-config.yaml \
  --host 127.0.0.1 \
  --port 4000
