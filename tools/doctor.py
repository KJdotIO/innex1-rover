#!/usr/bin/env python3
"""Rover doctor: setup and runtime preflight checks.

Usage:
  python3 tools/doctor.py                 # setup checks by default
  python3 tools/doctor.py --mode setup
  python3 tools/doctor.py --mode runtime
  python3 tools/doctor.py --mode all      # setup + runtime (runtime auto-skips if stack is not up)
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List


ROOT = Path(__file__).resolve().parents[1]


@dataclass
class CheckResult:
    status: str  # PASS | WARN | FAIL
    name: str
    detail: str
    suggestion: str = ""


def run_cmd(cmd: List[str], timeout: int = 8) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            cmd,
            returncode=124,
            stdout=exc.stdout or "",
            stderr=(exc.stderr or "") + "\ncommand timed out",
        )


def check_python_version() -> CheckResult:
    major, minor = sys.version_info[:2]
    if (major, minor) >= (3, 10):
        return CheckResult("PASS", "Python version", f"Detected {major}.{minor}")
    return CheckResult(
        "FAIL",
        "Python version",
        f"Detected {major}.{minor}, expected >= 3.10",
        "Use Python 3.10+ (RoboStack/ROS Humble friendly).",
    )


def check_command_exists(command: str, label: str) -> CheckResult:
    if shutil.which(command):
        return CheckResult("PASS", label, f"Found '{command}' in PATH")
    return CheckResult(
        "FAIL",
        label,
        f"'{command}' not found in PATH",
        f"Install/activate environment that provides '{command}'.",
    )


def check_ros_distro() -> CheckResult:
    distro = os.environ.get("ROS_DISTRO", "").strip()
    if not distro:
        return CheckResult(
            "WARN",
            "ROS environment",
            "ROS_DISTRO is not set",
            "Source ROS setup script, e.g. 'source /opt/ros/humble/setup.bash'.",
        )
    return CheckResult("PASS", "ROS environment", f"ROS_DISTRO={distro}")


def check_path_exists(path: Path, label: str) -> CheckResult:
    if path.exists():
        return CheckResult("PASS", label, str(path))
    return CheckResult(
        "FAIL",
        label,
        f"Missing: {path}",
        f"Verify file/package install and regenerate if needed: {path}",
    )


def check_open3d_available() -> CheckResult:
    if importlib.util.find_spec("open3d"):
        return CheckResult("PASS", "Python dependency: open3d", "Module import available")
    return CheckResult(
        "WARN",
        "Python dependency: open3d",
        "Module not importable in current interpreter",
        "Install in active env, e.g. 'python3 -m pip install open3d' (or skip if code path no longer needs it).",
    )


def check_ros_graph_available() -> CheckResult:
    proc = run_cmd(["ros2", "topic", "list"], timeout=8)
    if proc.returncode == 0:
        return CheckResult("PASS", "ROS graph access", "ros2 topic list succeeded")
    return CheckResult(
        "WARN",
        "ROS graph access",
        (proc.stderr or proc.stdout or "unable to query ROS graph").strip(),
        "Start/sourcing your ROS environment and launch stack before runtime checks.",
    )


def check_required_topics() -> CheckResult:
    proc = run_cmd(["ros2", "topic", "list"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Required topics",
            "Skipped: ROS graph unavailable",
            "Run after launching navigation stack.",
        )

    seen = set(t.strip() for t in proc.stdout.splitlines() if t.strip())
    required = {
        "/odometry/filtered",
        "/hazards/front",
        "/camera_front/points",
        "/tf",
        "/tf_static",
    }
    missing = sorted(required - seen)
    if not missing:
        return CheckResult("PASS", "Required topics", "All required topics found")
    return CheckResult(
        "WARN",
        "Required topics",
        f"Missing topics: {', '.join(missing)}",
        "Check bringup launch and producer nodes before mission run.",
    )


def check_mission_action_servers() -> CheckResult:
    proc = run_cmd(["ros2", "action", "list"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Mission action servers",
            "Skipped: ROS graph unavailable",
            "Run after launching the full stack.",
        )

    seen = set(a.strip() for a in proc.stdout.splitlines() if a.strip())
    required = {"/mission/excavate", "/mission/deposit"}
    missing = sorted(required - seen)
    if not missing:
        return CheckResult("PASS", "Mission action servers", "excavate and deposit servers found")
    return CheckResult(
        "WARN",
        "Mission action servers",
        f"Missing action servers: {', '.join(missing)}",
        "Launch material_action_server before starting a mission run.",
    )


def check_nav2_lifecycle() -> CheckResult:
    nodes_proc = run_cmd(["ros2", "lifecycle", "nodes"], timeout=8)
    if nodes_proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Nav2 lifecycle",
            "Skipped: unable to query lifecycle nodes",
            "Run after Nav2 launch (lifecycle manager active).",
        )

    active_expect = ["/bt_navigator", "/planner_server", "/controller_server"]
    missing_nodes = [n for n in active_expect if n not in nodes_proc.stdout]
    if missing_nodes:
        return CheckResult(
            "WARN",
            "Nav2 lifecycle",
            f"Missing lifecycle nodes: {', '.join(missing_nodes)}",
            "Confirm nav2_bringup launch and node names.",
        )

    not_active = []
    for node in active_expect:
        state_proc = run_cmd(["ros2", "lifecycle", "get", node], timeout=8)
        output = (state_proc.stdout + state_proc.stderr).lower()
        if state_proc.returncode != 0 or "active" not in output:
            not_active.append(node)

    if not not_active:
        return CheckResult("PASS", "Nav2 lifecycle", "Core Nav2 nodes report active")

    return CheckResult(
        "WARN",
        "Nav2 lifecycle",
        f"Nodes not active: {', '.join(not_active)}",
        "Use lifecycle manager/autostart or manually transition nodes to active.",
    )


def run_checks(checks: List[Callable[[], CheckResult]]) -> List[CheckResult]:
    results: List[CheckResult] = []
    for check in checks:
        try:
            results.append(check())
        except Exception as exc:  # pylint: disable=broad-except
            results.append(
                CheckResult(
                    "FAIL",
                    check.__name__,
                    f"Unexpected error: {exc}",
                    "Inspect stack trace and rerun.",
                )
            )
    return results


def print_results(results: List[CheckResult], heading: str) -> None:
    print(f"\n== {heading} ==")
    for r in results:
        print(f"[{r.status}] {r.name}: {r.detail}")
        if r.suggestion:
            print(f"  -> {r.suggestion}")


def final_exit_code(results: List[CheckResult]) -> int:
    if any(r.status == "FAIL" for r in results):
        return 2
    if any(r.status == "WARN" for r in results):
        return 1
    return 0


def should_run_runtime_checks() -> bool:
    """Decide whether runtime checks are meaningful right now.

    Runtime checks are intended for a launched stack. If nothing relevant is up,
    we skip them in default/all mode to avoid noisy false alarms.
    """
    proc = run_cmd(["ros2", "node", "list"], timeout=8)
    if proc.returncode != 0:
        return False

    nodes = set(n.strip() for n in proc.stdout.splitlines() if n.strip())
    runtime_markers = {
        "/bt_navigator",
        "/planner_server",
        "/controller_server",
        "/hazard_detection",
    }
    return any(marker in nodes for marker in runtime_markers)


def main() -> int:
    parser = argparse.ArgumentParser(description="Lunabotics rover doctor checks")
    parser.add_argument(
        "--mode",
        choices=["setup", "runtime", "all"],
        default="setup",
        help="Which checks to run",
    )
    args = parser.parse_args()

    setup_checks: List[Callable[[], CheckResult]] = [
        check_python_version,
        lambda: check_command_exists("ros2", "ROS CLI"),
        lambda: check_command_exists("colcon", "Colcon"),
        check_ros_distro,
        lambda: check_path_exists(ROOT / "src", "Workspace src/ directory"),
        lambda: check_path_exists(
            ROOT / "src/lunabot_navigation/config/nav2_params.yaml",
            "Nav2 config",
        ),
        lambda: check_path_exists(
            ROOT / "src/lunabot_navigation/behavior_trees/mission_navigate_to_pose_bt.xml",
            "Mission BT XML",
        ),
        lambda: check_path_exists(
            ROOT / ".github/contracts/interface_contracts.json",
            "Interface contracts JSON",
        ),
        lambda: check_path_exists(
            ROOT / "src/lunabot_mission/lunabot_mission/readiness_gate.py",
            "Mission readiness gate node",
        ),
        check_open3d_available,
    ]

    runtime_checks: List[Callable[[], CheckResult]] = [
        check_ros_graph_available,
        check_required_topics,
        check_nav2_lifecycle,
        check_mission_action_servers,
    ]

    all_results: List[CheckResult] = []

    if args.mode in {"setup", "all"}:
        setup_results = run_checks(setup_checks)
        print_results(setup_results, "Setup checks")
        all_results.extend(setup_results)

    if args.mode == "runtime":
        runtime_results = run_checks(runtime_checks)
        print_results(runtime_results, "Runtime checks")
        all_results.extend(runtime_results)
    elif args.mode == "all":
        if should_run_runtime_checks():
            runtime_results = run_checks(runtime_checks)
            print_results(runtime_results, "Runtime checks")
            all_results.extend(runtime_results)
        else:
            skip_msg = CheckResult(
                "PASS",
                "Runtime checks",
                "Skipped (no active nav/runtime nodes detected)",
                "Run with '--mode runtime' after bringup to validate live graph health.",
            )
            print_results([skip_msg], "Runtime checks")
            all_results.append(skip_msg)

    exit_code = final_exit_code(all_results)
    summary = "PASS" if exit_code == 0 else "WARN" if exit_code == 1 else "FAIL"
    print(f"\nOverall: {summary}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
