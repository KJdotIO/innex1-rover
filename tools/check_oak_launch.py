#!/usr/bin/env python3
"""Smoke-check the installed OAK-D Pro launch path without needing the camera."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys


def _run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run one subprocess command and echo it for easy debugging."""
    print(f"$ {' '.join(command)}")
    result = subprocess.run(
        command,
        check=False,
        text=True,
        capture_output=True,
    )
    if result.stdout:
        print(result.stdout.rstrip())
    if result.stderr:
        print(result.stderr.rstrip(), file=sys.stderr)
    if check and result.returncode != 0:
        raise RuntimeError(
            f"Command failed with exit code {result.returncode}: {' '.join(command)}"
        )
    return result


def _check_ros2_available() -> None:
    """Make sure the smoke test is being run in a ROS shell."""
    if shutil.which("ros2") is not None:
        return
    raise RuntimeError(
        "ros2 was not found on PATH. Source Humble and the workspace before "
        "running this smoke test."
    )


def _check_package(package_name: str) -> None:
    """Ensure a required ROS package is installed in the active environment."""
    _run(["ros2", "pkg", "prefix", package_name])


def _check_show_args(profile: str) -> None:
    """Make sure the installed launch file can be parsed by ros2 launch."""
    _run(
        [
            "ros2",
            "launch",
            "lunabot_sensors",
            "oakd_pro.launch.py",
            "profile:=" + profile,
            "--show-args",
        ]
    )


def _check_invalid_profile_rejected() -> None:
    """Confirm the wrapper still fails early on a bad profile value."""
    result = _run(
        [
            "ros2",
            "launch",
            "lunabot_sensors",
            "oakd_pro.launch.py",
            "profile:=definitely_wrong",
        ],
        check=False,
    )
    combined_output = "\n".join(
        part for part in [result.stdout, result.stderr] if part
    )
    if result.returncode == 0:
        raise RuntimeError("Invalid profile unexpectedly succeeded.")
    if "Unsupported OAK-D Pro profile" not in combined_output:
        raise RuntimeError(
            "Invalid profile failed, but not with the expected validation message."
        )


def _check_runtime_start(profile: str, timeout_s: int) -> None:
    """Start the launch briefly and accept the expected no-device boundary."""
    result = _run(
        [
            "timeout",
            str(timeout_s),
            "ros2",
            "launch",
            "lunabot_sensors",
            "oakd_pro.launch.py",
            "profile:=" + profile,
        ],
        check=False,
    )
    combined_output = "\n".join(
        part for part in [result.stdout, result.stderr] if part
    )
    acceptable = (
        result.returncode == 124
        or "Cannot find any device with given deviceInfo" in combined_output
    )
    if acceptable:
        return
    raise RuntimeError(
        "Runtime smoke test failed before reaching the expected timeout or "
        "the expected no-device boundary."
    )


def main() -> int:
    """Run the OAK launch smoke checks."""
    parser = argparse.ArgumentParser(
        description="Smoke-check the installed OAK-D Pro launch path."
    )
    parser.add_argument(
        "--profile",
        default="usb2_degraded",
        choices=["usb2_degraded", "usb3_full"],
        help="Hardware profile to validate.",
    )
    parser.add_argument(
        "--runtime",
        action="store_true",
        help="Start the launch briefly and accept a timeout or no-device error.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=15,
        help="Timeout in seconds for the optional runtime smoke test.",
    )
    args = parser.parse_args()

    _check_ros2_available()
    _check_package("lunabot_sensors")
    _check_package("depthai_ros_driver_v3")
    _check_show_args(args.profile)
    _check_invalid_profile_rejected()

    if args.runtime:
        _check_runtime_start(args.profile, args.timeout)

    print("OAK launch smoke check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
