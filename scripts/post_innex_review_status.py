#!/usr/bin/env python3
"""Post lightweight Nexy status comments without starting a model review."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys


STATUS_MARKER = "<!-- codex-rover-status -->"


def run(args: list[str], *, input_text: str | None = None) -> str:
    result = subprocess.run(args, input=input_text, capture_output=True, text=True)
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise SystemExit(f"Command failed: {' '.join(args)}\n{message}")
    return result.stdout


def env(name: str, default: str = "") -> str:
    return os.environ.get(name, default)


def help_body() -> str:
    return "\n".join(
        [
            STATUS_MARKER,
            "## Nexy Help",
            "",
            "Here are the commands I understand:",
            "",
            "| Command | What it does | Starts the model? |",
            "| --- | --- | --- |",
            "| `/nexy` or `/innex` | Reviews the PR, unless this commit has already been reviewed. | Sometimes |",
            "| `/nexy please focus on launch files` | Reviews the PR and treats the extra text as guidance. | Sometimes |",
            "| `/nexy status` or `/innex status` | Shows the latest Nexy review state and reviewed commit. | No |",
            "| `/nexy force review` or `/innex force review` | Runs a fresh review even if this commit was already reviewed. | Yes |",
            "| `/nexy help` or `/innex help` | Shows this help message. | No |",
            "",
            "I only request changes for high-confidence P0/P1 blockers. Smaller notes stay non-blocking.",
        ]
    ).strip() + "\n"


def build_body(mode: str) -> str:
    if mode == "help":
        return help_body()

    latest_state = env("LATEST_REVIEW_STATE", "none")
    latest_author = env("LATEST_REVIEW_AUTHOR", "none")
    latest_time = env("LATEST_REVIEW_SUBMITTED_AT")
    latest_commit = env("LATEST_REVIEW_COMMIT_SHORT")
    head_sha = env("HEAD_SHA_SHORT")

    lines = [STATUS_MARKER, "## Nexy Status", ""]
    if mode == "already-reviewed":
        lines.extend(
            [
                f"Already reviewed this head commit (`{head_sha}`). No model run was started.",
                "",
                "Use `/nexy force review` or `/innex force review` if you want a fresh review anyway.",
            ]
        )
        return "\n".join(lines).strip() + "\n"

    if latest_state == "none":
        lines.append("No previous Nexy review was found for this PR.")
    else:
        lines.extend(
            [
                f"Latest Nexy review: **{latest_state.lower()}** by `{latest_author}`.",
                f"Reviewed commit: `{latest_commit or 'unknown'}`.",
            ]
        )
        if latest_time:
            lines.append(f"Submitted: `{latest_time}`.")
        if latest_commit and latest_commit == head_sha:
            lines.append("")
            lines.append("Current head is already covered by that review.")
        else:
            lines.append("")
            lines.append(f"Current head is `{head_sha}` and has not been reviewed by Nexy yet.")
    return "\n".join(lines).strip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=env("GITHUB_REPOSITORY"))
    parser.add_argument("--pr", default=env("PR_NUMBER"))
    parser.add_argument("--mode", choices={"status", "already-reviewed", "help"}, required=True)
    args = parser.parse_args()
    if not args.repo:
        raise SystemExit("--repo or GITHUB_REPOSITORY is required")
    if not args.pr:
        raise SystemExit("--pr or PR_NUMBER is required")

    body = build_body(args.mode)
    run(
        [
            "gh",
            "api",
            f"repos/{args.repo}/issues/{args.pr}/comments",
            "--method",
            "POST",
            "--field",
            f"body={body}",
        ]
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
