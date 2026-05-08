#!/usr/bin/env bash
set -euo pipefail

repo="${NEXY_REPO:-KJdotIO/innex1-rover}"
nexy_home="${NEXY_HOME:-${HOME}/nexy/innex1-rover-review}"
runner_home="${NEXY_RUNNER_HOME:-${HOME}/actions-runners/innex1-rover}"
runner_name="${NEXY_RUNNER_NAME:-$(hostname -s)-rover-review}"

usage() {
  cat <<USAGE
Usage: $0 [--configure-runner]

Installs the laptop-local Nexy runtime outside any external-drive checkout.

Environment overrides:
  NEXY_REPO          GitHub repo, default: ${repo}
  NEXY_HOME          LiteLLM wrapper/config directory, default: ${nexy_home}
  NEXY_RUNNER_HOME   GitHub runner install directory, default: ${runner_home}
  NEXY_RUNNER_NAME   Runner name, default: ${runner_name}

Options:
  --configure-runner  Download/configure the GitHub Actions runner.
  -h, --help          Show this help.
USAGE
}

configure_runner=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --configure-runner)
      configure_runner=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_command gh
require_command python3
require_command curl
require_command tar

gh auth status >/dev/null

mkdir -p "${nexy_home}"
cp .github/codex/litellm-config.yaml "${nexy_home}/litellm-config.yaml"

cat > "${nexy_home}/start_litellm_rover_review.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export VIBEPROXY_API_KEY="${VIBEPROXY_API_KEY:-sk-vibeproxy-local}"
export LITELLM_MASTER_KEY="${LITELLM_MASTER_KEY:-sk-local-rover-review}"

uvx --from 'litellm[proxy]' litellm \
  --config "${script_dir}/litellm-config.yaml" \
  --host 127.0.0.1 \
  --port 4000
SH
chmod +x "${nexy_home}/start_litellm_rover_review.sh"

if [ "${configure_runner}" = "1" ]; then
  require_command uname
  arch="$(uname -m)"
  if [ "${arch}" != "arm64" ]; then
    echo "This installer currently expects macOS arm64; got ${arch}." >&2
    exit 1
  fi

  mkdir -p "${runner_home}"

  if [ ! -x "${runner_home}/config.sh" ]; then
    tmp_dir="$(mktemp -d)"
    cleanup() {
      rm -rf "${tmp_dir}"
    }
    trap cleanup EXIT

    asset_url="$(python3 - <<'PY'
import json
import urllib.request

with urllib.request.urlopen("https://api.github.com/repos/actions/runner/releases/latest") as response:
    release = json.load(response)

for asset in release["assets"]:
    name = asset["name"]
    if name.startswith("actions-runner-osx-arm64-") and name.endswith(".tar.gz"):
        print(asset["browser_download_url"])
        break
else:
    raise SystemExit("No osx-arm64 runner asset found in latest release")
PY
)"
    curl -L "${asset_url}" -o "${tmp_dir}/actions-runner-osx-arm64.tar.gz"
    tar -xzf "${tmp_dir}/actions-runner-osx-arm64.tar.gz" -C "${runner_home}"
  fi

  token="$(gh api -X POST "repos/${repo}/actions/runners/registration-token" --jq .token)"
  (
    cd "${runner_home}"
    ./config.sh \
      --unattended \
      --url "https://github.com/${repo}" \
      --token "${token}" \
      --name "${runner_name}" \
      --work _work \
      --labels rover-review \
      --replace
  )
fi

cat <<EOF
Nexy laptop runtime is ready.

LiteLLM:
  ${nexy_home}/start_litellm_rover_review.sh

Runner:
  cd ${runner_home}
  ./run.sh

The runner workspace will live under:
  ${runner_home}/_work
EOF
