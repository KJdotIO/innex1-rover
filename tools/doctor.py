#!/usr/bin/env python3
"""Rover doctor: setup and runtime preflight checks.

Usage:
  python3 tools/doctor.py                         # developer setup checks
  python3 tools/doctor.py --profile operator      # laptop / operator access checks
  python3 tools/doctor.py --profile jetson        # rover-side checks
  python3 tools/doctor.py --mode runtime          # live ROS graph checks
  python3 tools/doctor.py --mode all              # setup + runtime
  python3 tools/doctor.py --profile operator --fix
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError:  # Let the doctor explain how to install PyYAML.
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_CONFIG_PATH = ROOT / "src/lunabot_bringup/config/preflight_checks.yaml"
ROVER_IP = "192.168.8.20"
_PREFLIGHT_CACHE = None
_PREFLIGHT_ERROR = None

BANNER = r"""
██╗███╗   ██╗███╗   ██╗███████╗██╗  ██╗      ██╗
██║████╗  ██║████╗  ██║██╔════╝╚██╗██╔╝     ███║
██║██╔██╗ ██║██╔██╗ ██║█████╗   ╚███╔╝█████╗╚██║
██║██║╚██╗██║██║╚██╗██║██╔══╝   ██╔██╗╚════╝ ██║
██║██║ ╚████║██║ ╚████║███████╗██╔╝ ██╗      ██║
╚═╝╚═╝  ╚═══╝╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝      ╚═╝

██████╗  ██████╗  ██████╗████████╗ ██████╗ ██████╗
██╔══██╗██╔═══██╗██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗
██║  ██║██║   ██║██║        ██║   ██║   ██║██████╔╝
██║  ██║██║   ██║██║        ██║   ██║   ██║██╔══██╗
██████╔╝╚██████╔╝╚██████╗   ██║   ╚██████╔╝██║  ██║
╚═════╝  ╚═════╝  ╚═════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝
"""


@dataclass
class CheckResult:
    status: str  # PASS | WARN | FAIL
    name: str
    detail: str
    suggestion: str = ""

    def as_dict(self) -> dict[str, str]:
        return {
            "status": self.status,
            "name": self.name,
            "detail": self.detail,
            "suggestion": self.suggestion,
        }


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
        if yaml is None:
            raise ModuleNotFoundError("PyYAML is not installed")
        data = yaml.safe_load(PREFLIGHT_CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("preflight"), dict):
            raise ValueError("missing top-level 'preflight' mapping")
        preflight = data["preflight"]
        _validate_preflight_config(preflight)
        _PREFLIGHT_CACHE = preflight

        return _PREFLIGHT_CACHE, None
    except Exception as exc:  # pylint: disable=broad-except
        _PREFLIGHT_CACHE = None
        _PREFLIGHT_ERROR = str(exc)
        return None, _PREFLIGHT_ERROR


def _validate_preflight_config(config: dict) -> None:
    """Validate the doctor-facing preflight config shape."""
    required_sections = {
        "required_topics": {"name"},
        "required_nodes": {"name"},
        "required_actions": {"name"},
        "required_tf_links": {"parent", "child"},
    }
    for section, required_keys in required_sections.items():
        if section not in config:
            raise ValueError(f"missing preflight.{section}")
        section_items = config[section]
        if not isinstance(section_items, list):
            raise ValueError(f"preflight.{section} must be a list")
        for index, item in enumerate(section_items):
            if not isinstance(item, dict):
                raise ValueError(f"preflight.{section}[{index}] must be a mapping")
            missing_keys = required_keys - item.keys()
            if missing_keys:
                missing = ", ".join(sorted(missing_keys))
                raise ValueError(
                    f"preflight.{section}[{index}] missing keys: {missing}"
                )


def _preflight_error_result(name: str) -> CheckResult:
    _config, error = _load_preflight_config()
    suggestion = (
        "Fix the preflight config or doctor will drift from the real readiness checks."
    )
    if error == "PyYAML is not installed":
        venv_python = _venv_python_with_module("yaml")
        if venv_python is not None:
            suggestion = f"Use the existing environment: {venv_python} tools/doctor.py"
    return CheckResult(
        "WARN",
        name,
        f"Skipped: could not load {PREFLIGHT_CONFIG_PATH.name} ({error})",
        suggestion,
    )


def _configured_required_names(section: str, field: str = "name") -> set[str] | None:
    config, error = _load_preflight_config()
    if error is not None or config is None:
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
    if error is not None or config is None:
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


def run_cmd(cmd: list[str], timeout: int = 8) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd,
            capture_output=True,
            cwd=ROOT,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            cmd,
            returncode=124,
            stdout=_coerce_output(exc.stdout),
            stderr=_coerce_output(exc.stderr) + "\ncommand timed out",
        )
    except FileNotFoundError as exc:
        return subprocess.CompletedProcess(
            cmd,
            returncode=127,
            stdout="",
            stderr=f"command not found: {exc.filename or cmd[0]}",
        )
    except PermissionError as exc:
        return subprocess.CompletedProcess(
            cmd,
            returncode=126,
            stdout="",
            stderr=f"permission denied: {exc.filename or cmd[0]}",
        )


def _command(command: list[str]) -> str:
    return " ".join(command)


def _venv_python_with_module(module: str) -> Path | None:
    for candidate in (ROOT / ".venv/bin/python", ROOT / ".venv-quality/bin/python"):
        if not candidate.exists():
            continue
        proc = run_cmd(
            [
                str(candidate),
                "-c",
                f"import importlib.util; raise SystemExit(importlib.util.find_spec('{module}') is None)",
            ],
            timeout=5,
        )
        if proc.returncode == 0:
            return candidate
    return None


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


def check_command_exists(
    command: str, label: str, *, required: bool = True, suggestion: str | None = None
) -> CheckResult:
    if shutil.which(command):
        return CheckResult("PASS", label, f"Found '{command}' in PATH")
    status = "FAIL" if required else "WARN"
    return CheckResult(
        status,
        label,
        f"'{command}' not found in PATH",
        suggestion or f"Install/activate environment that provides '{command}'.",
    )


def check_platform_profile(profile: str) -> CheckResult:
    system = platform.system()
    machine = platform.machine()
    detail = f"{system} {platform.release()} ({machine})"

    if profile == "operator":
        return CheckResult(
            "PASS",
            "Host platform",
            detail,
            "Operator laptops do not need ROS locally when using Foxglove or the Jetson-hosted web Gamepad page.",
        )

    if system == "Linux":
        return CheckResult("PASS", "Host platform", detail)

    return CheckResult(
        "WARN",
        "Host platform",
        detail,
        "ROS 2 Humble development is expected on Ubuntu 22.04, the Jetson, a VM, or the ROS CI image.",
    )


def check_git_repo() -> CheckResult:
    proc = run_cmd(["git", "rev-parse", "--show-toplevel"], timeout=5)
    if proc.returncode != 0:
        return CheckResult(
            "FAIL",
            "Git workspace",
            "This directory is not a Git checkout",
            f"Run doctor from the rover repo root: {ROOT}",
        )

    top = Path(proc.stdout.strip()).resolve()
    if top == ROOT:
        return CheckResult("PASS", "Git workspace", str(top))
    return CheckResult(
        "WARN",
        "Git workspace",
        f"Git root is {top}, expected {ROOT}",
        "Run from the rover repo root so relative paths and package checks are meaningful.",
    )


def check_git_status() -> CheckResult:
    proc = run_cmd(["git", "status", "--short"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Git status",
            "Could not read Git status",
            "Check whether the checkout is healthy before editing or deploying.",
        )
    changed = [line for line in proc.stdout.splitlines() if line.strip()]
    if not changed:
        return CheckResult("PASS", "Git status", "Working tree is clean")
    return CheckResult(
        "WARN",
        "Git status",
        f"{len(changed)} changed path(s) in working tree",
        "Read the diff before deploying; local hardware notes may be uncommitted.",
    )


def check_disk_space() -> CheckResult:
    usage = shutil.disk_usage(ROOT)
    free_gib = usage.free / (1024**3)
    if free_gib >= 10:
        return CheckResult("PASS", "Disk space", f"{free_gib:.1f} GiB free")
    if free_gib >= 5:
        return CheckResult(
            "WARN",
            "Disk space",
            f"{free_gib:.1f} GiB free",
            "ROS builds, bags and Docker layers can fill this quickly.",
        )
    return CheckResult(
        "FAIL",
        "Disk space",
        f"{free_gib:.1f} GiB free",
        "Free disk space before building, recording bags, or pulling images.",
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
    if distro != "humble":
        return CheckResult(
            "WARN",
            "ROS environment",
            f"ROS_DISTRO={distro}",
            "This rover stack targets ROS 2 Humble; source Humble unless you are deliberately testing a port.",
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
        return CheckResult(
            "PASS", "Python dependency: open3d", "Module import available"
        )
    return CheckResult(
        "WARN",
        "Python dependency: open3d",
        "Module not importable in current interpreter",
        "Install in active env, e.g. 'python3 -m pip install open3d' (or skip if code path no longer needs it).",
    )


def check_python_module(module: str, label: str, *, required: bool) -> CheckResult:
    if importlib.util.find_spec(module):
        return CheckResult("PASS", label, "Module import available")
    status = "FAIL" if required else "WARN"
    venv_python = _venv_python_with_module(module)
    if venv_python is not None:
        suggestion = f"Use the existing environment: {venv_python} tools/doctor.py"
    else:
        suggestion = f"Install the preflight requirements: {_command([sys.executable, '-m', 'pip', 'install', '-r', '.github/requirements-preflight.txt'])}"
    return CheckResult(
        status,
        label,
        "Module not importable in current interpreter",
        suggestion,
    )


def check_docker_cli() -> CheckResult:
    return check_command_exists(
        "docker",
        "Docker CLI",
        required=False,
        suggestion="Install Docker Desktop or Docker Engine if this laptop will build/run the rover dev container.",
    )


def check_docker_daemon() -> CheckResult:
    if not shutil.which("docker"):
        return CheckResult(
            "WARN",
            "Docker daemon",
            "Skipped: docker CLI is not installed",
            "Install Docker before running container checks.",
        )
    proc = run_cmd(["docker", "info", "--format", "{{.ServerVersion}}"], timeout=8)
    if proc.returncode == 0:
        return CheckResult("PASS", "Docker daemon", f"Server {proc.stdout.strip()}")
    return CheckResult(
        "WARN",
        "Docker daemon",
        (proc.stderr or proc.stdout or "docker daemon not reachable").strip(),
        "Start Docker Desktop/Engine before building or running operator images.",
    )


def check_ssh_client() -> CheckResult:
    return check_command_exists(
        "ssh",
        "SSH client",
        required=False,
        suggestion="Install OpenSSH client so operators can reach the Jetson when Foxglove or the web Gamepad page is not enough.",
    )


def check_tailscale_cli() -> CheckResult:
    return check_command_exists(
        "tailscale",
        "Tailscale CLI",
        required=False,
        suggestion="Install Tailscale if the team uses it for remote Jetson access.",
    )


def check_jetson_host() -> CheckResult:
    machine = platform.machine()
    nv_tegra = Path("/etc/nv_tegra_release")
    if nv_tegra.exists():
        return CheckResult("PASS", "Jetson host", nv_tegra.read_text().strip())
    if machine in {"aarch64", "arm64"}:
        return CheckResult(
            "WARN",
            "Jetson host",
            f"ARM host detected ({machine}) but /etc/nv_tegra_release is missing",
            "If this is not the Jetson, use '--profile operator' for laptop checks.",
        )
    return CheckResult(
        "WARN",
        "Jetson host",
        f"Not detected on this machine ({machine})",
        "Run '--profile jetson' on the rover Jetson for hardware-side setup checks.",
    )


def check_serial_devices() -> CheckResult:
    devices = sorted(Path("/dev").glob("ttyACM*")) + sorted(
        Path("/dev").glob("ttyUSB*")
    )
    if devices:
        return CheckResult(
            "PASS",
            "Serial devices",
            ", ".join(str(device) for device in devices),
        )
    return CheckResult(
        "WARN",
        "Serial devices",
        "No /dev/ttyACM* or /dev/ttyUSB* devices found",
        "Connect the Teensy or motor controller path before drivetrain hardware tests.",
    )


def check_rover_lan_ip() -> CheckResult:
    if not shutil.which("ip"):
        return CheckResult(
            "WARN",
            "Rover LAN IP",
            "Skipped: 'ip' command not found",
            "On the Jetson, confirm the rover LAN address manually.",
        )
    proc = run_cmd(["ip", "-br", "addr", "show"], timeout=5)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Rover LAN IP",
            "Could not query network interfaces",
            "Run 'ip -br addr show' on the Jetson and check the router LAN address.",
        )
    if ROVER_IP in proc.stdout:
        return CheckResult("PASS", "Rover LAN IP", f"{ROVER_IP} is assigned")
    return CheckResult(
        "WARN",
        "Rover LAN IP",
        f"{ROVER_IP} not found on local interfaces",
        "The current router and Ouster debug config expect the Jetson on 192.168.8.20.",
    )


def check_router_config_consistency() -> CheckResult:
    files = {
        "router": ROOT / "docs/hardware/gl-a1300-router.md",
        "foxglove": ROOT / "docs/foxglove/README.md",
        "ouster": ROOT / "src/lunabot_bringup/config/ouster_lidar_debug.yaml",
    }
    missing = [name for name, path in files.items() if not path.exists()]
    if missing:
        return CheckResult(
            "WARN",
            "Router IP docs",
            f"Skipped missing files: {', '.join(missing)}",
            "Restore the router, Foxglove and Ouster config docs before competition readiness checks.",
        )
    stale = [
        name
        for name, path in files.items()
        if ROVER_IP not in path.read_text(encoding="utf-8")
    ]
    if not stale:
        return CheckResult(
            "PASS", "Router IP docs", f"All checked files mention {ROVER_IP}"
        )
    return CheckResult(
        "WARN",
        "Router IP docs",
        f"{ROVER_IP} missing from: {', '.join(stale)}",
        "Keep router docs, Foxglove docs and Ouster udp_dest aligned before competition testing.",
    )


def check_preflight_config_load() -> CheckResult:
    """Preflight config load check.
    Verify the preflight config exists and matches the expected schema.
    """
    _, error = _load_preflight_config()
    if error is None:
        return CheckResult(
            "PASS", "Preflight config load", f"Loaded {PREFLIGHT_CONFIG_PATH.name}"
        )
    suggestion = (
        "Fix the preflight config schema or syntax before relying on doctor results."
    )
    if error == "PyYAML is not installed":
        venv_python = _venv_python_with_module("yaml")
        if venv_python is not None:
            suggestion = f"Use the existing environment: {venv_python} tools/doctor.py"
    return CheckResult(
        "FAIL",
        "Preflight config load",
        f"Could not load {PREFLIGHT_CONFIG_PATH.name}: {error}",
        suggestion,
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
    required_topics = {_normalize_ros_name(name) for name in required_topics}

    seen = {_normalize_ros_name(t) for t in proc.stdout.splitlines() if t.strip()}
    missing = sorted(required_topics - seen)
    if not missing:
        return CheckResult("PASS", "Required topics", "All required topics found")
    return CheckResult(
        "WARN",
        "Required topics",
        f"Missing topics: {', '.join(missing)}",
        "Check bringup launch and producer nodes before mission run.",
    )


def check_safety_topics() -> CheckResult:
    proc = run_cmd(["ros2", "topic", "list"], timeout=8)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Safety topics",
            "Skipped: ROS graph unavailable",
            "Run after launching safety and drivetrain stacks.",
        )

    required = {"safety/estop", "safety/motion_inhibit"}
    seen = {_normalize_ros_name(t) for t in proc.stdout.splitlines() if t.strip()}
    missing = sorted(required - seen)
    if not missing:
        return CheckResult(
            "PASS",
            "Safety topics",
            "/safety/estop and /safety/motion_inhibit found",
        )
    return CheckResult(
        "FAIL",
        "Safety topics",
        f"Missing safety topic(s): {', '.join(missing)}",
        "Do not run hardware motion until the safety stack publishes these topics.",
    )


def check_operator_ports() -> CheckResult:
    if not shutil.which("ss"):
        return CheckResult(
            "WARN",
            "Operator bridge ports",
            "Skipped: 'ss' command not found",
            "Check Foxglove and web Gamepad ports manually.",
        )
    proc = run_cmd(["ss", "-ltn"], timeout=5)
    if proc.returncode != 0:
        return CheckResult(
            "WARN",
            "Operator bridge ports",
            "Could not query listening TCP ports",
            "Run 'ss -ltn' on the Jetson.",
        )
    found = []
    missing = []
    for port, label in (("8765", "Foxglove"), ("9443", "web Gamepad")):
        port_pattern = re.compile(rf"(?<!\d):{re.escape(port)}(?!\d)")
        if any(port_pattern.search(line) for line in proc.stdout.splitlines()):
            found.append(f"{label}:{port}")
        else:
            missing.append(f"{label}:{port}")
    if not missing:
        return CheckResult("PASS", "Operator bridge ports", ", ".join(found))
    return CheckResult(
        "WARN",
        "Operator bridge ports",
        f"Listening: {', '.join(found) if found else 'none'}; missing: {', '.join(missing)}",
        "Start Foxglove and/or the HTTPS web Gamepad bridge before operator checks.",
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

    seen = {_normalize_ros_name(n) for n in proc.stdout.splitlines() if n.strip()}
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
    required_actions = {_normalize_ros_name(name) for name in required_actions}

    seen = {_normalize_ros_name(a) for a in proc.stdout.splitlines() if a.strip()}
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
        if not any(
            token in output for token in ["translation:", "rotation:", "at time"]
        ):
            missing.append(f"{parent}->{child}")

    if not missing:
        return CheckResult(
            "PASS", "Required TF links", "All required TF links resolved"
        )
    return CheckResult(
        "WARN",
        "Required TF links",
        f"Missing TF links: {', '.join(missing)}",
        "Check localisation, robot_state_publisher, and frame names before mission run.",
    )


def run_checks(checks: list[Callable[[], CheckResult]]) -> list[CheckResult]:
    results: list[CheckResult] = []
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


def print_results(
    results: list[CheckResult], heading: str, *, json_output: bool
) -> None:
    if json_output:
        return
    print(f"\n== {heading} ==")
    for r in results:
        print(f"[{r.status}] {r.name}: {r.detail}")
        if r.suggestion:
            print(f"  -> {r.suggestion}")


def final_exit_code(results: list[CheckResult]) -> int:
    if any(r.status == "FAIL" for r in results):
        return 2
    if any(r.status == "WARN" for r in results):
        return 1
    return 0


def safe_repairs() -> list[CheckResult]:
    global yaml  # pylint: disable=global-statement
    repairs: list[CheckResult] = []

    if yaml is None and (ROOT / ".github/requirements-preflight.txt").exists():
        venv_python = _venv_python_with_module("yaml")
        if venv_python is not None:
            repairs.append(
                CheckResult(
                    "WARN",
                    "Repair: Python preflight deps",
                    "Skipped current interpreter install because an existing repo environment already has PyYAML",
                    f"Run: {venv_python} tools/doctor.py",
                )
            )
            return repairs

        command = [
            sys.executable,
            "-m",
            "pip",
            "install",
            "-r",
            ".github/requirements-preflight.txt",
        ]
        proc = run_cmd(command, timeout=180)
        status = "PASS" if proc.returncode == 0 else "FAIL"
        if proc.returncode == 0:
            yaml = importlib.import_module("yaml")
        repairs.append(
            CheckResult(
                status,
                "Repair: Python preflight deps",
                "Installed .github/requirements-preflight.txt"
                if proc.returncode == 0
                else (proc.stderr or proc.stdout or "pip install failed").strip(),
                "" if proc.returncode == 0 else f"Run manually: {_command(command)}",
            )
        )

    if not repairs:
        repairs.append(
            CheckResult(
                "PASS",
                "Repairs",
                "No safe local repairs were needed or available",
                "System-level installs such as ROS, Docker and Tailscale are reported as commands rather than run with sudo.",
            )
        )
    return repairs


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

    nodes = {_normalize_ros_name(n) for n in proc.stdout.splitlines() if n.strip()}
    return any(marker in nodes for marker in runtime_markers), None


def main() -> int:
    parser = argparse.ArgumentParser(description="Lunabotics rover doctor checks")
    parser.add_argument(
        "--mode",
        choices=["setup", "runtime", "all"],
        default="setup",
        help="Which checks to run",
    )
    parser.add_argument(
        "--profile",
        choices=["developer", "operator", "jetson", "ci", "all"],
        default="developer",
        help="Setup profile to check",
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Run safe local repairs only: Python preflight dependencies.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON instead of text.",
    )
    args = parser.parse_args()

    common_setup_checks: list[Callable[[], CheckResult]] = [
        check_python_version,
        lambda: check_platform_profile(args.profile),
        lambda: check_command_exists("git", "Git"),
        check_git_repo,
        check_git_status,
        check_disk_space,
        lambda: check_path_exists(ROOT / "src", "Workspace src/ directory"),
        lambda: check_path_exists(
            ROOT / ".github/contracts/interface_contracts.json",
            "Interface contracts JSON",
        ),
        check_router_config_consistency,
    ]

    preflight_setup_checks: list[Callable[[], CheckResult]] = [
        lambda: check_path_exists(
            ROOT / "src/lunabot_navigation/config/nav2_params.yaml",
            "Nav2 config",
        ),
        lambda: check_path_exists(
            ROOT
            / "src/lunabot_navigation/behavior_trees/navigate_to_pose_bounded_recovery.xml",
            "Nav2 bounded-recovery BT XML",
        ),
        lambda: check_path_exists(
            PREFLIGHT_CONFIG_PATH,
            "Preflight config",
        ),
        lambda: check_python_module("yaml", "Python dependency: PyYAML", required=True),
        check_preflight_config_load,
    ]

    ros_setup_checks: list[Callable[[], CheckResult]] = [
        lambda: check_command_exists("ros2", "ROS CLI"),
        lambda: check_command_exists("colcon", "Colcon"),
        lambda: check_command_exists("rosdep", "rosdep"),
        lambda: check_command_exists("vcs", "vcstool", required=False),
        check_ros_distro,
        check_open3d_available,
    ]

    operator_setup_checks: list[Callable[[], CheckResult]] = [
        check_docker_cli,
        check_docker_daemon,
        check_ssh_client,
        check_tailscale_cli,
    ]

    jetson_setup_checks: list[Callable[[], CheckResult]] = [
        check_jetson_host,
        check_rover_lan_ip,
        check_serial_devices,
        check_ssh_client,
        check_tailscale_cli,
    ]

    setup_checks = list(common_setup_checks)
    if args.profile in {"developer", "ci", "all"}:
        setup_checks.extend(preflight_setup_checks)
        setup_checks.extend(ros_setup_checks)
    if args.profile in {"operator", "all"}:
        if args.profile == "operator":
            setup_checks.append(
                lambda: check_python_module(
                    "yaml", "Python dependency: PyYAML", required=False
                )
            )
        setup_checks.extend(operator_setup_checks)
    if args.profile == "jetson":
        setup_checks.extend(preflight_setup_checks)
        setup_checks.extend(ros_setup_checks)
        setup_checks.extend(jetson_setup_checks)
    elif args.profile == "all":
        setup_checks.extend(jetson_setup_checks)

    runtime_checks: list[Callable[[], CheckResult]] = [
        check_preflight_config_load,
        check_ros_graph_available,
        check_safety_topics,
        check_required_topics,
        check_required_nodes,
        check_required_actions,
        check_required_tf_links,
        check_nav2_lifecycle,
        check_operator_ports,
    ]

    all_results: list[CheckResult] = []
    grouped_results: dict[str, list[dict[str, str]]] = {}

    if not args.json:
        print(BANNER)
        print("INNEX-1 Doctor: repo, robot and operator checks")

    if args.fix and args.mode in {"setup", "all"}:
        repair_results = safe_repairs()
        print_results(repair_results, "Repairs", json_output=args.json)
        grouped_results["repairs"] = [result.as_dict() for result in repair_results]
        all_results.extend(repair_results)

    if args.mode in {"setup", "all"}:
        setup_results = run_checks(setup_checks)
        print_results(setup_results, "Setup checks", json_output=args.json)
        grouped_results["setup"] = [result.as_dict() for result in setup_results]
        all_results.extend(setup_results)

    if args.mode == "runtime":
        runtime_results = run_checks(runtime_checks)
        print_results(runtime_results, "Runtime checks", json_output=args.json)
        grouped_results["runtime"] = [result.as_dict() for result in runtime_results]
        all_results.extend(runtime_results)
    elif args.mode == "all":
        should_run, runtime_skip_result = should_run_runtime_checks()
        if should_run:
            runtime_results = run_checks(runtime_checks)
            print_results(runtime_results, "Runtime checks", json_output=args.json)
            grouped_results["runtime"] = [
                result.as_dict() for result in runtime_results
            ]
            all_results.extend(runtime_results)
        elif runtime_skip_result is not None:
            print_results(
                [runtime_skip_result], "Runtime checks", json_output=args.json
            )
            grouped_results["runtime"] = [runtime_skip_result.as_dict()]
            all_results.append(runtime_skip_result)
        else:
            skip_msg = CheckResult(
                "PASS",
                "Runtime checks",
                "Skipped (no active nav/runtime nodes detected)",
                "Run with '--mode runtime' after bringup to validate live graph health.",
            )
            print_results([skip_msg], "Runtime checks", json_output=args.json)
            grouped_results["runtime"] = [skip_msg.as_dict()]
            all_results.append(skip_msg)

    exit_code = final_exit_code(all_results)
    summary = "PASS" if exit_code == 0 else "WARN" if exit_code == 1 else "FAIL"
    if args.json:
        print(
            json.dumps(
                {
                    "overall": summary,
                    "exit_code": exit_code,
                    "mode": args.mode,
                    "profile": args.profile,
                    "results": grouped_results,
                },
                indent=2,
            )
        )
    else:
        print(f"\nOverall: {summary}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
