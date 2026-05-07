#!/usr/bin/env bash
set -euo pipefail

manifest_path="${1:-.github/innex/app-manifest.json}"
owner="${GITHUB_OWNER:-KJdotIO}"
state="innex-review-$(date +%s)"

if [ ! -f "${manifest_path}" ]; then
  echo "Manifest not found: ${manifest_path}" >&2
  exit 1
fi

tmp_html="$(mktemp "${TMPDIR:-/tmp}/innex-app-manifest.XXXXXX.html")"
account_type="$(gh api "users/${owner}" --jq '.type' 2>/dev/null || echo User)"

python3 - "${manifest_path}" "${owner}" "${account_type}" "${state}" "${tmp_html}" <<'PY'
import html
import json
import sys
from pathlib import Path

manifest_path, owner, account_type, state, tmp_html = sys.argv[1:]
manifest = json.dumps(json.load(open(manifest_path, encoding="utf-8")))
if account_type == "Organization":
    action = f"https://github.com/organizations/{owner}/settings/apps/new?state={state}"
else:
    action = f"https://github.com/settings/apps/new?state={state}"
Path(tmp_html).write_text(
    "\n".join(
        [
            "<!doctype html>",
            "<meta charset='utf-8'>",
            "<title>Create INX One GitHub App</title>",
            "<form id='manifest' method='post' action='" + html.escape(action) + "'>",
            "<input type='hidden' name='manifest' value='" + html.escape(manifest, quote=True) + "'>",
            "</form>",
            "<script>document.getElementById('manifest').submit()</script>",
            "<p>Submitting INX One GitHub App manifest to GitHub...</p>",
        ]
    ),
    encoding="utf-8",
)
PY

echo "Opening GitHub App manifest flow for ${owner} (${account_type})."
echo "After GitHub redirects, copy the 'code=' value and run:"
echo "  scripts/convert_innex_github_app_manifest.sh CODE"
open "${tmp_html}"
