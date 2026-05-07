#!/usr/bin/env bash
set -euo pipefail

code="${1:-}"
if [ -z "${code}" ]; then
  echo "Usage: $0 MANIFEST_CODE" >&2
  exit 1
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
