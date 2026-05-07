#!/usr/bin/env python3
"""Post a structured Codex review as a GitHub pull request review."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


SEVERITY_ORDER = {"P0": 0, "P1": 1, "P2": 2}
SEVERITY_LABELS = {
    "P0": "P0 Critical",
    "P1": "P1 Major",
    "P2": "P2 Minor",
}
SEVERITY_BADGES = {
    "P0": "🔴",
    "P1": "🟠",
    "P2": "🟡",
}
EFFORT_LABELS = {
    "quick_win": "Quick win",
    "moderate": "Moderate",
    "heavy_lift": "Heavy lift",
}


def run(args: list[str]) -> str:
    result = subprocess.run(args, capture_output=True, text=True)
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise SystemExit(f"Command failed: {' '.join(args)}\n{message}")
    return result.stdout


def load_review(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8").strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Could not parse Codex review JSON: {exc}") from exc
    validate_review(data)
    return data


def validate_review(data: dict[str, Any]) -> None:
    required = {
        "review_mode",
        "summary",
        "merge_assessment",
        "decision",
        "findings",
        "checks_run",
        "docs_consulted",
        "risks_not_checked",
    }
    missing = sorted(required.difference(data))
    if missing:
        raise SystemExit(f"Review JSON is missing required keys: {', '.join(missing)}")
    if data["decision"] not in {"comment", "request_changes", "approve"}:
        raise SystemExit("Review decision must be comment, request_changes, or approve")
    if not isinstance(data["findings"], list):
        raise SystemExit("Review findings must be a list")
    for index, finding in enumerate(data["findings"], start=1):
        for key in (
            "severity",
            "category",
            "effort",
            "confidence",
            "title",
            "file",
            "line",
            "body",
            "evidence",
            "suggested_fix",
            "references",
        ):
            if key not in finding:
                raise SystemExit(f"Finding {index} is missing {key}")
        if finding["severity"] not in SEVERITY_ORDER:
            raise SystemExit(f"Finding {index} has unknown severity {finding['severity']!r}")
        if finding["effort"] not in EFFORT_LABELS:
            raise SystemExit(f"Finding {index} has unknown effort {finding['effort']!r}")
        confidence = finding["confidence"]
        if not isinstance(confidence, int | float) or not 0 <= confidence <= 1:
            raise SystemExit(f"Finding {index} confidence must be between 0 and 1")
        if finding["severity"] in {"P0", "P1"} and confidence < 0.8:
            raise SystemExit(f"Finding {index} is blocking but confidence is below 0.8")
        if not isinstance(finding["line"], int) or finding["line"] < 1:
            raise SystemExit(f"Finding {index} line must be a positive integer")


def github_json(endpoint: str) -> Any:
    return json.loads(run(["gh", "api", endpoint]))


def changed_lines_from_patch(patch: str | None) -> set[int]:
    if not patch:
        return set()
    changed: set[int] = set()
    new_line = 0
    hunk_re = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")
    for line in patch.splitlines():
        match = hunk_re.match(line)
        if match:
            new_line = int(match.group(1))
            continue
        if line.startswith("+") and not line.startswith("+++"):
            changed.add(new_line)
            new_line += 1
        elif line.startswith("-") and not line.startswith("---"):
            continue
        elif line.startswith("\\"):
            continue
        else:
            new_line += 1
    return changed


def get_pr_files(repo: str, pr_number: int) -> dict[str, set[int]]:
    files: dict[str, set[int]] = {}
    page = 1
    while True:
        batch = github_json(
            f"repos/{repo}/pulls/{pr_number}/files?per_page=100&page={page}"
        )
        if not batch:
            break
        for item in batch:
            files[item["filename"]] = changed_lines_from_patch(item.get("patch"))
        page += 1
    return files


def table_cell(value: Any) -> str:
    return str(value).replace("\n", " ").replace("|", "\\|").strip()


def optional_env_int(name: str) -> int | None:
    value = os.environ.get(name)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer") from exc


def details_block(summary: str, lines: list[str]) -> list[str]:
    return [
        "",
        "<details>",
        f"<summary>{summary}</summary>",
        "",
        *lines,
        "",
        "</details>",
    ]


def format_finding(finding: dict[str, Any], *, inline: bool) -> str:
    label = SEVERITY_LABELS[finding["severity"]]
    badge = SEVERITY_BADGES[finding["severity"]]
    body = (
        f"**{badge} {label}: {finding['title']}**\n\n"
        f"_Category: {finding['category']} · Effort: {EFFORT_LABELS[finding['effort']]} · "
        f"Confidence: {finding['confidence']:.0%}_\n\n"
        f"{finding['body'].strip()}\n\n"
        f"**Evidence:** {finding['evidence'].strip()}\n\n"
        f"**Suggested fix:** {finding['suggested_fix'].strip()}"
    )
    references = finding.get("references") or []
    if references:
        body += "\n\nReferences:\n"
        body += "\n".join(f"- {item}" for item in references)
    suggestion = finding.get("suggestion")
    if inline and suggestion:
        body += f"\n\n```suggestion\n{suggestion.rstrip()}\n```"
    return body


def build_review_body(
    review: dict[str, Any],
    non_inline_findings: list[dict[str, Any]],
    skipped_inline_findings: list[dict[str, Any]],
) -> str:
    findings = sorted(
        review["findings"], key=lambda item: (SEVERITY_ORDER[item["severity"]], item["file"])
    )
    counts = {severity: 0 for severity in SEVERITY_ORDER}
    for finding in findings:
        counts[finding["severity"]] += 1

    lines = [
        "<!-- codex-rover-review -->",
        "## Nexy Review",
        "",
    ]

    has_blocker = any(
        finding["severity"] in {"P0", "P1"} and finding["confidence"] >= 0.8
        for finding in findings
    )
    if has_blocker:
        blocker_count = counts["P0"] + counts["P1"]
        blocker_label = "blocker" if blocker_count == 1 else "blockers"
        lines.extend(
            [
                "> [!CAUTION]",
                f"> **Changes requested:** {blocker_count} high-confidence {blocker_label} found. These should be fixed before merging.",
                "",
            ]
        )
    elif findings:
        lines.extend(
            [
                "> [!NOTE]",
                "> **Mergeable from Nexy's review standpoint.** The notes below are non-blocking.",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "> [!TIP]",
                "> ✅ **LGTM from Nexy. No blocking issues found; mergeable from review standpoint.** 🎉",
                "",
            ]
        )

    lines.extend(
        [
            review["summary"].strip(),
            "",
            f"**Merge assessment:** {review['merge_assessment'].strip()}",
            "",
        ]
    )

    lines.extend(
        [
            "| Severity | Meaning | Count |",
            "| --- | --- | ---: |",
            f"| 🔴 P0 | Critical blocker | {counts['P0']} |",
            f"| 🟠 P1 | Major issue | {counts['P1']} |",
            f"| 🟡 P2 | Minor or follow-up | {counts['P2']} |",
        ]
    )

    if findings:
        lines.extend(["", "| Finding | Location | Effort | Confidence | Why it matters |", "| --- | --- | --- | ---: | --- |"])
        for finding in findings:
            location = f"`{finding['file']}:{finding['line']}`"
            label = f"{SEVERITY_BADGES[finding['severity']]} {SEVERITY_LABELS[finding['severity']]}"
            title = table_cell(finding["title"])
            body = table_cell(finding["body"])
            effort = EFFORT_LABELS[finding["effort"]]
            confidence = f"{finding['confidence']:.0%}"
            lines.append(f"| {label}: {title} | {location} | {effort} | {confidence} | {body} |")
    if review["checks_run"]:
        lines.extend(details_block("Checks run", [f"- {item}" for item in review["checks_run"]]))

    if review["docs_consulted"]:
        lines.extend(details_block("Docs consulted", [f"- {item}" for item in review["docs_consulted"]]))

    if review["risks_not_checked"]:
        lines.extend(details_block("Risks not checked", [f"- {item}" for item in review["risks_not_checked"]]))

    if non_inline_findings or skipped_inline_findings:
        non_inline_lines = []
        for finding in [*non_inline_findings, *skipped_inline_findings]:
            location = f"{finding['file']}:{finding['line']}"
            non_inline_lines.append(
                f"- **{finding['severity']} {finding['title']}** at `{location}`: "
                f"{finding['body'].strip()}"
            )
            references = finding.get("references") or []
            if references:
                non_inline_lines.extend(f"  Reference: {item}" for item in references)
        lines.extend(details_block("Findings not posted inline", non_inline_lines))

    return "\n".join(lines).strip() + "\n"


def review_event(decision: str, findings: list[dict[str, Any]]) -> str:
    has_blocker = any(
        finding["severity"] in {"P0", "P1"} and finding["confidence"] >= 0.8
        for finding in findings
    )
    if has_blocker:
        return "REQUEST_CHANGES"
    return "COMMENT"


def post_review(
    repo: str,
    pr_number: int,
    review: dict[str, Any],
    *,
    dry_run: bool,
) -> None:
    pr = github_json(f"repos/{repo}/pulls/{pr_number}")
    commit_id = pr["head"]["sha"]
    changed_files = get_pr_files(repo, pr_number)

    comments: list[dict[str, Any]] = []
    non_inline: list[dict[str, Any]] = []
    skipped_inline: list[dict[str, Any]] = []

    for finding in review["findings"]:
        path = finding["file"]
        line = finding["line"]
        if path in changed_files and line in changed_files[path]:
            comments.append(
                {
                    "path": path,
                    "line": line,
                    "side": "RIGHT",
                    "body": format_finding(finding, inline=True),
                }
            )
        elif path in changed_files:
            skipped_inline.append(finding)
        else:
            non_inline.append(finding)

    payload = {
        "commit_id": commit_id,
        "event": review_event(review["decision"], review["findings"]),
        "body": build_review_body(review, non_inline, skipped_inline),
        "comments": comments,
    }

    if dry_run:
        print(json.dumps(payload, indent=2))
        return

    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as handle:
        json.dump(payload, handle)
        handle.flush()
        payload_path = handle.name

    try:
        run(
            [
                "gh",
                "api",
                f"repos/{repo}/pulls/{pr_number}/reviews",
                "--method",
                "POST",
                "--input",
                payload_path,
            ]
        )
    finally:
        Path(payload_path).unlink(missing_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("review_json", type=Path)
    parser.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY"))
    parser.add_argument(
        "--pr",
        type=int,
        default=optional_env_int("PR_NUMBER"),
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.repo:
        raise SystemExit("--repo or GITHUB_REPOSITORY is required")
    if not args.pr:
        raise SystemExit("--pr or PR_NUMBER is required")
    review = load_review(args.review_json)
    post_review(args.repo, int(args.pr), review, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
