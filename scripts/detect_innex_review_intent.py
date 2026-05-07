#!/usr/bin/env python3
"""Decide whether a Nexy review request should run the model."""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path
from typing import Any


MARKER = "<!-- codex-rover-review -->"
FORCE_RE = re.compile(r"\b(force|again|fresh|full|deep|re-?review|rerun)\b", re.I)


def append_output(name: str, value: str) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        print(f"{name}={value}")
        return
    with open(output_path, "a", encoding="utf-8") as handle:
        handle.write(f"{name}={value}\n")


def load_reviews(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return json.loads(path.read_text(encoding="utf-8"))


def innex_reviews(reviews: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        review for review in reviews
        if MARKER in (review.get("body") or "")
    ]


def short_sha(value: str) -> str:
    return value[:7] if value else ""


def has_command(comment: str, suffix: str) -> bool:
    return f"/innex {suffix}" in comment or f"/nexy {suffix}" in comment


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reviews", type=Path, required=True)
    parser.add_argument("--head-sha", required=True)
    parser.add_argument("--comment", default="")
    args = parser.parse_args()

    comment = args.comment or ""
    lower_comment = comment.lower()
    status_requested = has_command(lower_comment, "status")
    help_requested = has_command(lower_comment, "help")
    force_requested = bool(FORCE_RE.search(comment))

    reviews = innex_reviews(load_reviews(args.reviews))
    latest = reviews[-1] if reviews else {}
    latest_commit = latest.get("commit_id") or ""
    same_head = bool(latest_commit and latest_commit == args.head_sha)
    already_reviewed = (
        bool(comment.strip())
        and same_head
        and not status_requested
        and not help_requested
        and not force_requested
    )

    append_output("status", str(status_requested).lower())
    append_output("help", str(help_requested).lower())
    append_output("already_reviewed", str(already_reviewed).lower())
    append_output("force_requested", str(force_requested).lower())
    append_output("latest_review_id", str(latest.get("id") or ""))
    append_output("latest_review_state", str(latest.get("state") or "none"))
    append_output("latest_review_author", str((latest.get("user") or {}).get("login") or "none"))
    append_output("latest_review_submitted_at", str(latest.get("submitted_at") or ""))
    append_output("latest_review_commit", latest_commit)
    append_output("latest_review_commit_short", short_sha(latest_commit))
    append_output("head_sha_short", short_sha(args.head_sha))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
