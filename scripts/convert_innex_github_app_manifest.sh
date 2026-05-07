#!/usr/bin/env bash
set -euo pipefail

input="${1:-}"
if [ -z "${input}" ]; then
  echo "Usage: $0 MANIFEST_CODE_OR_REDIRECT_URL" >&2
  exit 1
fi

if [ "${input}" = "CODE" ] || [ "${input}" = "CODE_FROM_URL" ] || [ "${input}" = "PASTE_REDIRECT_URL_OR_CODE_HERE" ]; then
  echo "Replace ${input} with the real temporary code from GitHub's redirect URL." >&2
  exit 1
fi

code="${input}"
if [[ "${input}" == http://* || "${input}" == https://* ]]; then
  code="$(python3 - "${input}" <<'PY'
import sys
from urllib.parse import parse_qs, urlparse

query = parse_qs(urlparse(sys.argv[1]).query)
values = query.get("code")
if not values or not values[0]:
    raise SystemExit("Could not find a code= value in the supplied URL")
print(values[0])
PY
)"
fi

output="${INNEX_APP_OUTPUT:-innex-app-conversion.json}"
gh api "app-manifests/${code}/conversions" --method POST > "${output}"

echo "Wrote ${output}. It contains the app private key, so do not commit it."
echo
echo "Set these repository secrets from the JSON:"
echo "  INNEX_APP_ID          = .id"
echo "  INNEX_APP_PRIVATE_KEY = .pem"
echo
echo "Then install the app on KJdotIO/innex1-rover and upload docs/assets/innex-app-avatar.png as the app avatar in GitHub settings."
