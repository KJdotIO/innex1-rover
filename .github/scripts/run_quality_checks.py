#!/usr/bin/env python3
"""Run repository quality checks for Python, YAML, and workflow files."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BLOCKING_PYTHON_TARGETS = [
    ".github/scripts",
]
ADVISORY_PYTHON_TARGETS = [
    ".github/scripts",
    "tools",
    "src/lunabot_bringup",
    "src/lunabot_control",
    "src/lunabot_excavation",
    "src/lunabot_localisation",
]
BLOCKING_YAML_TARGETS = [
    ".github",
    ".pre-commit-config.yaml",
    ".yamllint.yml",
]
COMPLEXITY_TARGETS = [
    "tools",
    "src/lunabot_bringup",
    "src/lunabot_control",
    "src/lunabot_excavation",
    "src/lunabot_localisation",
]


def _run(command: list[str]) -> int:
    result = subprocess.run(command, cwd=REPO_ROOT)
    return result.returncode


def _blocking_commands() -> list[list[str]]:
    return [
        [
            sys.executable,
            "-m",
            "ruff",
            "check",
            *BLOCKING_PYTHON_TARGETS,
            "--output-format",
            "concise",
            "--select",
            "E,F,W",
            "--ignore",
            "E501",
        ],
        [sys.executable, "-m", "ruff", "format", "--check", *BLOCKING_PYTHON_TARGETS],
        [sys.executable, "-m", "yamllint", *BLOCKING_YAML_TARGETS],
    ]


def _advisory_commands() -> list[list[str]]:
    return [
        [
            sys.executable,
            "-m",
            "ruff",
            "check",
            *ADVISORY_PYTHON_TARGETS,
            "--output-format",
            "concise",
            "--select",
            "I,B,UP,SIM,RET,ARG,PTH,RUF,C4",
            "--ignore",
            "B008",
        ],
        [sys.executable, "-m", "pyright"],
        [
            sys.executable,
            "-m",
            "xenon",
            "--max-absolute",
            "C",
            "--max-modules",
            "B",
            "--max-average",
            "B",
            *COMPLEXITY_TARGETS,
        ],
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("blocking", "advisory"),
        required=True,
        help="Select the quality gate set to run.",
    )
    args = parser.parse_args()

    commands = _blocking_commands() if args.mode == "blocking" else _advisory_commands()

    failures = 0
    for command in commands:
        print(f"Running: {' '.join(command)}")
        failures += _run(command)

    if failures:
        print(f"{args.mode.capitalize()} quality checks failed.")
        return 1

    print(f"{args.mode.capitalize()} quality checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
