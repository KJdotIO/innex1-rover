#!/usr/bin/env bash
set -euo pipefail

BASE_REF="${BASE_REF:-origin/main}"
OUTPUT_FILE="${OUTPUT_FILE:-codex-review.json}"
PROMPT_FILE=".github/codex/prompts/rover-review.md"
SCHEMA_FILE=".github/codex/schemas/rover-review.schema.json"
CODEX_RESPONSES_API_ENDPOINT="${CODEX_RESPONSES_API_ENDPOINT:-http://127.0.0.1:4000/v1/responses}"
LITELLM_REVIEW_KEY="${LITELLM_REVIEW_KEY:-sk-local-rover-review}"
created_wiki=0
created_diff=0
proxy_port=""

tmp_dir="$(mktemp -d)"
cleanup() {
  if [ -n "${proxy_port:-}" ]; then
    curl -sf "http://127.0.0.1:${proxy_port}/shutdown" >/dev/null 2>&1 || true
  elif [ -n "${proxy_pid:-}" ]; then
    kill "$proxy_pid" >/dev/null 2>&1 || true
  fi
  if [ "$created_wiki" = "1" ]; then
    rm -rf wiki
  fi
  if [ "$created_diff" = "1" ]; then
    rm -f "$PR_DIFF_FILE"
  fi
  rm -rf "$tmp_dir"
}
trap cleanup EXIT

if [ ! -d wiki ]; then
  git clone https://github.com/KJdotIO/innex1-rover.wiki.git wiki || true
  if [ -d wiki ]; then
    created_wiki=1
  fi
fi

git fetch origin main --quiet || true

PR_DIFF_FILE="${PR_DIFF_FILE:-codex-pr.diff}"
export PR_DIFF_FILE
if [ ! -f "$PR_DIFF_FILE" ]; then
  git diff --find-renames "$BASE_REF"...HEAD > "$PR_DIFF_FILE"
  created_diff=1
fi

export PR_NUMBER="${PR_NUMBER:-0}"
export PR_TITLE="${PR_TITLE:-Local rover review}"
export PR_BODY="${PR_BODY:-Local review against ${BASE_REF}}"
export PR_BASE_REF="${PR_BASE_REF:-main}"
export PR_HEAD_REF="${PR_HEAD_REF:-$(git branch --show-current || echo HEAD)}"
export PR_BASE_SHA="${PR_BASE_SHA:-$(git rev-parse "$BASE_REF")}"
export PR_HEAD_SHA="${PR_HEAD_SHA:-$(git rev-parse HEAD)}"
export GITHUB_EVENT_NAME="${GITHUB_EVENT_NAME:-local}"
export REVIEW_REQUEST_BODY="${REVIEW_REQUEST_BODY:-}"

server_info="$tmp_dir/server-info.json"
(
  printf '%s' "$LITELLM_REVIEW_KEY" |
    npx -y @openai/codex-responses-api-proxy \
      --http-shutdown \
      --server-info "$server_info" \
      --upstream-url "$CODEX_RESPONSES_API_ENDPOINT"
) &
proxy_pid=$!

for _ in {1..20}; do
  if [ -s "$server_info" ]; then
    break
  fi
  sleep 0.5
done

if [ ! -s "$server_info" ]; then
  echo "Codex Responses API proxy did not start." >&2
  exit 1
fi

proxy_port="$(python3 - "$server_info" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
print(data["port"])
PY
)"

codex_home="$tmp_dir/codex-home"
mkdir -p "$codex_home"
{
  echo 'model_provider = "codex-action-responses-proxy"'
  echo
  cat .github/codex/config/config.toml
  echo
  echo '[model_providers.codex-action-responses-proxy]'
  echo 'name = "Codex Action Responses Proxy"'
  echo "base_url = \"http://127.0.0.1:${proxy_port}/v1\""
  echo 'wire_api = "responses"'
} > "$codex_home/config.toml"

CODEX_HOME="$codex_home" codex exec \
  --sandbox read-only \
  --output-schema "$SCHEMA_FILE" \
  --output-last-message "$OUTPUT_FILE" \
  -c "model=\"${CODEX_REVIEW_MODEL:-rover-review}\"" \
  -c "model_reasoning_effort=\"${CODEX_REVIEW_REASONING_EFFORT:-high}\"" \
  -c "model_provider=\"codex-action-responses-proxy\"" \
  - < "$PROMPT_FILE"

python3 -m json.tool "$OUTPUT_FILE" >/dev/null
echo "Wrote $OUTPUT_FILE"
