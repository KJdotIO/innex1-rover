#!/usr/bin/env python3
"""Regression checks for CI package selection policy."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SELECTOR_PATH = REPO_ROOT / ".github/scripts/select_ci_packages.py"


def _load_selector_module():
    spec = importlib.util.spec_from_file_location("select_ci_packages", SELECTOR_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    selector = _load_selector_module()
    package_roots, dependencies = selector._discover_packages()

    scenarios = {
        "empty_diff_forces_full": (
            [],
            ("full", ()),
        ),
        "excavation_safe_leaf": (
            ["src/lunabot_excavation/lunabot_excavation/excavation_controller.py"],
            ("packages", ("lunabot_control", "lunabot_excavation")),
        ),
        "leaf_package": (
            ["src/lunabot_teleop/config/xbox_teleop.yaml"],
            ("packages", ("lunabot_bringup", "lunabot_teleop")),
        ),
        "interfaces_force_full": (
            ["src/lunabot_interfaces/msg/ExcavationStatus.msg"],
            ("full", ()),
        ),
        "workflow_force_full": (
            [".github/workflows/nightly-full-ci.yml"],
            ("full", ()),
        ),
        "external_force_full": (
            ["src/external/leo_common-ros2/leo_teleop/config/joy_config.yaml"],
            ("full", ()),
        ),
        "localisation_force_full": (
            ["src/lunabot_localisation/launch/localisation.launch.py"],
            ("full", ()),
        ),
        "description_force_full": (
            ["src/lunabot_description/urdf/lunabot.urdf.xacro"],
            ("full", ()),
        ),
    }

    failures: list[str] = []
    for name, (changed_files, expected) in scenarios.items():
        selection = selector._select(changed_files, package_roots, dependencies)
        actual = (selection.mode, selection.packages)
        if actual != expected:
            failures.append(
                f"{name}: expected {expected}, got {(selection.mode, selection.packages)}"
            )

    if failures:
        print("CI package selector regression checks failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    head_sha = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        check=True,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    ).stdout.strip()
    invalid_diff = selector._git_changed_files(
        "deadbeefdeadbeefdeadbeefdeadbeefdeadbeef",
        head_sha,
    )
    if invalid_diff != []:
        print("CI package selector regression checks failed:")
        print(f"- invalid_diff_range: expected [], got {invalid_diff}")
        return 1

    print("CI package selector regression checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
