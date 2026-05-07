#!/usr/bin/env bash
set -euo pipefail

root="${1:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}"

rm -rf \
  "${root}/.firecrawl" \
  "${root}/research/.firecrawl" \
  "${root}/wiki"

rm -f \
  "${root}/codex-pr.diff" \
  "${root}/codex-pr.json" \
  "${root}/codex-pr-reviews.json" \
  "${root}/codex-review-history.md" \
  "${root}/codex-review.json" \
  "${root}/innex-app-conversion.json"
