#!/usr/bin/env python3
"""Regression checks for CI package selection policy."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
import tempfile
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

    ros_ci_scenarios = {
        "docs_only_skips_ros_ci": (["README.md"], False),
        "src_change_needs_ros_ci": (
            ["src/lunabot_teleop/config/xbox_teleop.yaml"],
            True,
        ),
        "workflow_change_needs_ros_ci": ([".github/workflows/ci.yml"], True),
        "empty_diff_forces_ros_ci": ([], True),
    }

    scenarios = {
        "empty_diff_forces_full": (
            [],
            ("full", ()),
        ),
        "excavation_safe_leaf": (
            ["src/lunabot_excavation/lunabot_excavation/excavation_controller.py"],
            (
                "packages",
                ("lunabot_bringup", "lunabot_control", "lunabot_excavation"),
            ),
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
    for name, (changed_files, expected) in ros_ci_scenarios.items():
        actual = selector.requires_ros_ci(changed_files)
        if actual != expected:
            failures.append(f"{name}: expected {expected}, got {actual}")

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

    with tempfile.NamedTemporaryFile("r+", encoding="utf-8") as handle:
        old_git_changed_files = selector._git_changed_files
        old_argv = sys.argv[:]
        selector._git_changed_files = lambda _base, _head: ["README.md"]
        sys.argv = [
            "select_ci_packages.py",
            "--base-sha",
            "base",
            "--head-sha",
            "head",
            "--github-output",
            handle.name,
        ]
        try:
            result = selector.main()
        finally:
            selector._git_changed_files = old_git_changed_files
            sys.argv = old_argv

        if result != 0:
            print("CI package selector regression checks failed:")
            print(f"- cli_output: expected exit 0, got {result}")
            return 1

        output = handle.read()
        expected_fragments = [
            "ros_ci=false",
            "mode=skip",
            "packages=",
            "reason=ROS build/test not needed for this change set",
        ]
        missing = [
            fragment for fragment in expected_fragments if fragment not in output
        ]
        if missing:
            print("CI package selector regression checks failed:")
            for fragment in missing:
                print(f"- cli_output: missing {fragment!r} in {output!r}")
            return 1

    print("CI package selector regression checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
