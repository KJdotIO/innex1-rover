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
import yaml

ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_CONFIG_PATH = ROOT / "src/lunabot_bringup/config/preflight_checks.yaml"
_PREFLIGHT_CACHE = None
_PREFLIGHT_ERROR = None


@dataclass
class CheckResult:
    status: str  # PASS | WARN | FAIL
    name: str
    detail: str
    suggestion: str = ""


def _coerce_output(value: str | bytes | None) -> str:
    """Normalize subprocess output to text.

    `TimeoutExpired.stdout` / `.stderr` may still be bytes even when the
    original subprocess was launched with `text=True`.
    """
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _load_preflight_config():
    global _PREFLIGHT_CACHE, _PREFLIGHT_ERROR
    if _PREFLIGHT_CACHE is not None or _PREFLIGHT_ERROR is not None:
        return _PREFLIGHT_CACHE, _PREFLIGHT_ERROR

    try:
        data = yaml.safe_load(PREFLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("preflight"), dict):
            raise ValueError("missing top-level 'preflight' mapping")
        _PREFLIGHT_CACHE = data["preflight"]
        _validate_preflight_config(_PREFLIGHT_CACHE)
        return _PREFLIGHT_CACHE, None
    except Exception as exc:  # pylint: disable=broad-except
        _PREFLIGHT_ERROR = str(exc)
        return None, _PREFLIGHT_ERROR


def _validate_preflight_config(config: dict) -> None:
    """Validate the doctor-facing preflight config shape."""
    list_sections = (
        "required_topics",
        "required_nodes",
        "required_actions",
        "required_tf_links",
    )
    for section in list_sections:
        section_items = config.get(section, [])
        if not isinstance(section_items, list):
            raise ValueError(f"preflight.{section} must be a list")


def _preflight_error_result(name: str) -> CheckResult:
    _config, error = _load_preflight_config()
    return CheckResult(
        "WARN",
        name,
        f"Skipped: could not load {PREFLIGHT_CONFIG_PATH.name} ({error})",
        "Fix the preflight config or doctor will drift from the real readiness checks.",
    )


def _configured_required_names(section: str, field: str = "name") -> set[str] | None:
    config, error = _load_preflight_config()
    if error is not None:
        return None

    section_items = config.get(section, [])
    if not isinstance(section_items, list):
        return None

    names = set()
    for item in section_items:
        if not isinstance(item, dict):
            continue
        if not bool(item.get("critical", True)):
            continue
        value = item.get(field)
        if value:
            names.add(str(value))
    return names


def _configured_required_tf_links() -> list[tuple[str, str]] | None:
    config, error = _load_preflight_config()
    if error is not None:
        return None

    section_items = config.get("required_tf_links", [])
    if not isinstance(section_items, list):
        return None

    links = []
    for item in section_items:
        if not isinstance(item, dict):
            continue
        if not bool(item.get("critical", True)):
            continue
        parent = item.get("parent")
        child = item.get("child")
        if parent and child:
            links.append((str(parent), str(child)))
    return links


def _normalize_ros_name(name: str) -> str:
    """Normalize ROS graph names so config and CLI output compare consistently."""
    return name.strip().lstrip("/")


def run_cmd(cmd: List[str], timeout: int = 8) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            cmd,
            returncode=124,
            stdout=_coerce_output(exc.stdout),
            stderr=_coerce_output(exc.stderr) + "\ncommand timed out",
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


def check_preflight_config_load() -> CheckResult:
    """Verify that the preflight config exists and can be parsed into the expected shape."""
    _config, error = _load_preflight_config()
    if error is None:
        return CheckResult("PASS", "Preflight config load", f"Loaded {PREFLIGHT_CONFIG_PATH.name}")
    return CheckResult(
        "FAIL",
        "Preflight config load",
        f"Could not load {PREFLIGHT_CONFIG_PATH.name}: {error}",
        "Fix the preflight config schema or syntax before relying on doctor results.",
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

    required_topics = _configured_required_names("required_topics")
    if required_topics is None:
        return _preflight_error_result("Required topics")

    seen = set(t.strip() for t in proc.stdout.splitlines() if t.strip())
    missing = sorted(required_topics - seen)
    if not missing:
        return CheckResult("PASS", "Required topics", "All required topics found")
    return CheckResult(
        "WARN",
        "Required topics",
        f"Missing topics: {', '.join(missing)}",
        "Check bringup launch and producer nodes before mission run.",
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




def check_required_nodes() -> CheckResult:
    """Check required nodes.

    Verify that all critical configured nodes are present in the ROS graph.
    """
    proc = run_cmd(["ros2", "node", "list"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Required nodes",
            "Skipped: unable to query ROS nodes",
            "Run after launching navigation stack.",
        )

    required_nodes = _configured_required_names("required_nodes")
    if required_nodes is None:
        return _preflight_error_result("Required nodes")
    required_nodes = {_normalize_ros_name(name) for name in required_nodes}

    seen = set(_normalize_ros_name(n) for n in proc.stdout.splitlines() if n.strip())
    missing = sorted(required_nodes - seen)
    if not missing:
        return CheckResult("PASS", "Required nodes", "All required nodes found")
    return CheckResult(
        "WARN",
        "Required nodes",
        f"Missing nodes: {', '.join(missing)}",
        "Check bringup launch and crashed nodes before mission run.",
    )


def check_required_actions() -> CheckResult:
    """Check required actions.

    Verify that all critical configured action servers are currently available.
    """
    proc = run_cmd(["ros2", "action", "list"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Required actions",
            "Skipped: unable to query ROS actions",
            "Run after launching navigation stack.",
        )

    required_actions = _configured_required_names("required_actions")
    if required_actions is None:
        return _preflight_error_result("Required actions")

    seen = set(a.strip() for a in proc.stdout.splitlines() if a.strip())
    missing = sorted(required_actions - seen)
    if not missing:
        return CheckResult("PASS", "Required actions", "All required actions found")
    return CheckResult(
        "WARN",
        "Required actions",
        f"Missing actions: {', '.join(missing)}",
        "Check Nav2/action server startup before mission run.",
    )


def check_required_tf_links() -> CheckResult:
    """Check required TF links.

    Verify that all critical configured TF parent-child links can be resolved.
    """
    required_links = _configured_required_tf_links()
    if required_links is None:
        return _preflight_error_result("Required TF links")

    missing = []
    for parent, child in required_links:
        proc = run_cmd(["ros2", "run", "tf2_ros", "tf2_echo", parent, child], timeout=4)
        output = f"{proc.stdout}\n{proc.stderr}".lower()
        if not any(token in output for token in ["translation:", "rotation:", "at time"]):
            missing.append(f"{parent}->{child}")

    if not missing:
        return CheckResult("PASS", "Required TF links", "All required TF links resolved")
    return CheckResult(
        "WARN",
        "Required TF links",
        f"Missing TF links: {', '.join(missing)}",
        "Check localisation, robot_state_publisher, and frame names before mission run.",
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


def should_run_runtime_checks() -> tuple[bool, CheckResult | None]:
    """Decide whether runtime checks are meaningful right now.

    Runtime checks are intended for a launched stack. If nothing relevant is up,
    we skip them in default/all mode to avoid noisy false alarms.
    """
    proc = run_cmd(["ros2", "node", "list"], timeout=8)
    if proc.returncode != 0:
        return False, None

    runtime_markers = _configured_required_names("required_nodes")
    if runtime_markers is None:
        return False, _preflight_error_result("Runtime checks")
    runtime_markers = {_normalize_ros_name(name) for name in runtime_markers}

    nodes = set(_normalize_ros_name(n) for n in proc.stdout.splitlines() if n.strip())
    return any(marker in nodes for marker in runtime_markers), None


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
            PREFLIGHT_CONFIG_PATH,
            "Preflight config",
        ),
        check_preflight_config_load,
        lambda: check_path_exists(
            ROOT / ".github/contracts/interface_contracts.json",
            "Interface contracts JSON",
        ),
        check_open3d_available,
    ]

    runtime_checks: List[Callable[[], CheckResult]] = [
        check_preflight_config_load,
        check_ros_graph_available,
        check_required_topics,
        check_required_nodes,
        check_required_actions,
        check_required_tf_links,
        check_nav2_lifecycle,
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
        should_run, runtime_skip_result = should_run_runtime_checks()
        if should_run:
            runtime_results = run_checks(runtime_checks)
            print_results(runtime_results, "Runtime checks")
            all_results.extend(runtime_results)
        elif runtime_skip_result is not None:
            print_results([runtime_skip_result], "Runtime checks")
            all_results.append(runtime_skip_result)
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
