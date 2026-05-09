"""Validate lean Jetson runtime and communications profiles."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

COMPETITION_BUDGET_KBPS = 4000.0
COMPETITION_PROFILES = ("sim_competition", "hardware_competition")
RAW_TOPIC_SUFFIXES = ("/image", "/depth_image", "/points")
RAW_TOPIC_MARKERS = ("pointcloud",)


@dataclass(frozen=True)
class TelemetryTopic:
    """Expected topic rate and approximate payload size."""

    name: str
    max_hz: float
    estimated_bytes: int

    @property
    def estimated_kbps(self) -> float:
        """Return estimated average wire payload in Kbps."""
        return self.max_hz * self.estimated_bytes * 8.0 / 1000.0


@dataclass(frozen=True)
class RuntimeProfile:
    """One named runtime/comms profile."""

    name: str
    description: str
    budget_kbps: float
    target_kbps: float
    launch_arguments: dict[str, dict[str, Any]]
    required_processes: tuple[str, ...]
    optional_tools: tuple[str, ...]
    off_by_default: tuple[str, ...]
    foxglove_allowlist: tuple[str, ...]
    telemetry_topics: tuple[TelemetryTopic, ...]

    @property
    def estimated_kbps(self) -> float:
        """Return estimated average telemetry load in Kbps."""
        return sum(topic.estimated_kbps for topic in self.telemetry_topics)

    @property
    def budget_margin_kbps(self) -> float:
        """Return remaining budget after estimated telemetry load."""
        return self.budget_kbps - self.estimated_kbps


def default_profiles_path() -> Path:
    """Return the installed runtime profile config path."""
    from ament_index_python.packages import get_package_share_directory

    share_dir = Path(get_package_share_directory("lunabot_bringup"))
    return share_dir / "config" / "runtime_profiles.yaml"


def _strings(raw_values: Any, *, field: str, profile_name: str) -> tuple[str, ...]:
    if not isinstance(raw_values, list) or any(
        not isinstance(value, str) for value in raw_values
    ):
        raise ValueError(f"profile {profile_name!r} field {field!r} must be strings")
    return tuple(raw_values)


def _telemetry_topics(
    raw_values: Any, *, profile_name: str
) -> tuple[TelemetryTopic, ...]:
    if not isinstance(raw_values, list) or not raw_values:
        raise ValueError(f"profile {profile_name!r} must list telemetry topics")

    topics: list[TelemetryTopic] = []
    for item in raw_values:
        if not isinstance(item, dict):
            raise ValueError(f"profile {profile_name!r} has invalid topic entry")

        name = item.get("name")
        max_hz = item.get("max_hz")
        estimated_bytes = item.get("estimated_bytes")
        if not isinstance(name, str) or not name.startswith("/"):
            raise ValueError(f"profile {profile_name!r} has invalid topic name")
        if not isinstance(max_hz, int | float) or max_hz <= 0:
            raise ValueError(f"profile {profile_name!r} has invalid topic rate")
        if not isinstance(estimated_bytes, int) or estimated_bytes <= 0:
            raise ValueError(f"profile {profile_name!r} has invalid topic size")

        topics.append(
            TelemetryTopic(
                name=name,
                max_hz=float(max_hz),
                estimated_bytes=estimated_bytes,
            )
        )
    return tuple(topics)


def load_profiles(path: Path) -> dict[str, RuntimeProfile]:
    """Load runtime profiles from YAML."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("profiles"), dict):
        raise ValueError(f"invalid runtime profile file: {path}")

    profiles: dict[str, RuntimeProfile] = {}
    for name, raw_profile in data["profiles"].items():
        if not isinstance(name, str) or not isinstance(raw_profile, dict):
            raise ValueError(f"invalid runtime profile entry in {path}")

        launch_arguments = raw_profile.get("launch_arguments", {})
        if not isinstance(launch_arguments, dict):
            raise ValueError(f"profile {name!r} launch_arguments must be a mapping")

        profiles[name] = RuntimeProfile(
            name=name,
            description=str(raw_profile.get("description", "")),
            budget_kbps=float(raw_profile.get("budget_kbps", 0.0)),
            target_kbps=float(raw_profile.get("target_kbps", 0.0)),
            launch_arguments=launch_arguments,
            required_processes=_strings(
                raw_profile.get("required_processes", []),
                field="required_processes",
                profile_name=name,
            ),
            optional_tools=_strings(
                raw_profile.get("optional_tools", []),
                field="optional_tools",
                profile_name=name,
            ),
            off_by_default=_strings(
                raw_profile.get("off_by_default", []),
                field="off_by_default",
                profile_name=name,
            ),
            foxglove_allowlist=_strings(
                raw_profile.get("foxglove_allowlist", []),
                field="foxglove_allowlist",
                profile_name=name,
            ),
            telemetry_topics=_telemetry_topics(
                raw_profile.get("telemetry_topics"),
                profile_name=name,
            ),
        )
    return profiles


def validate_profile(profile: RuntimeProfile) -> list[str]:
    """Return validation errors for one runtime profile."""
    errors: list[str] = []
    if profile.estimated_kbps > profile.target_kbps:
        errors.append(
            f"{profile.name}: estimated telemetry {profile.estimated_kbps:.1f} "
            f"Kbps exceeds target {profile.target_kbps:.1f} Kbps"
        )
    if profile.estimated_kbps > profile.budget_kbps:
        errors.append(
            f"{profile.name}: estimated telemetry {profile.estimated_kbps:.1f} "
            f"Kbps exceeds budget {profile.budget_kbps:.1f} Kbps"
        )

    if profile.name in COMPETITION_PROFILES:
        if profile.budget_kbps > COMPETITION_BUDGET_KBPS:
            errors.append(
                f"{profile.name}: budget {profile.budget_kbps:.0f} Kbps exceeds "
                f"{COMPETITION_BUDGET_KBPS:.0f} Kbps"
            )

        forbidden = [
            topic
            for topic in profile.foxglove_allowlist
            if _is_raw_stream_topic(topic)
        ]
        if forbidden:
            errors.append(
                f"{profile.name}: competition Foxglove allowlist exposes raw streams: "
                f"{', '.join(forbidden)}"
            )

        off_by_default = " ".join(profile.off_by_default).lower()
        for marker in ("raw image", "raw pointcloud", "heavy rosbag"):
            if marker not in off_by_default:
                errors.append(f"{profile.name}: {marker} must be off by default")
    return errors


def _is_raw_stream_topic(topic: str) -> bool:
    """Return True when a topic is too heavy for competition Foxglove use."""
    normalised = topic.strip().lower()
    if normalised.endswith("/compressed") or normalised.endswith("/camera_info"):
        return False
    return normalised.endswith(RAW_TOPIC_SUFFIXES) or any(
        marker in normalised for marker in RAW_TOPIC_MARKERS
    )


def validate_profiles(profiles: dict[str, RuntimeProfile]) -> list[str]:
    """Return validation errors for all loaded profiles."""
    required = {
        "sim_debug",
        "sim_competition",
        "hardware_bringup",
        "hardware_competition",
    }
    errors = [
        f"missing required runtime profile: {name}"
        for name in sorted(required.difference(profiles))
    ]
    for profile in profiles.values():
        errors.extend(validate_profile(profile))
    return errors


def profile_summary(profile: RuntimeProfile) -> dict[str, Any]:
    """Return a JSON-serialisable profile summary."""
    return {
        "name": profile.name,
        "description": profile.description,
        "budget_kbps": profile.budget_kbps,
        "target_kbps": profile.target_kbps,
        "estimated_kbps": round(profile.estimated_kbps, 3),
        "budget_margin_kbps": round(profile.budget_margin_kbps, 3),
        "foxglove_allowlist": list(profile.foxglove_allowlist),
        "off_by_default": list(profile.off_by_default),
        "telemetry_topics": [
            {
                "name": topic.name,
                "max_hz": topic.max_hz,
                "estimated_bytes": topic.estimated_bytes,
                "estimated_kbps": round(topic.estimated_kbps, 3),
            }
            for topic in profile.telemetry_topics
        ],
    }


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect Jetson runtime and communications profiles.",
    )
    parser.add_argument("command", choices=("check", "show"))
    parser.add_argument("--profiles-file", type=Path, default=None)
    parser.add_argument("--profile", default="hardware_competition")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the runtime profile CLI."""
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    profiles = load_profiles(args.profiles_file or default_profiles_path())

    if args.command == "check":
        errors = validate_profiles(profiles)
        if errors:
            for error in errors:
                print(error, file=sys.stderr)
            return 1
        print("runtime profiles: pass")
        return 0

    profile = profiles.get(args.profile)
    if profile is None:
        print(f"unknown runtime profile: {args.profile}", file=sys.stderr)
        return 2

    summary = profile_summary(profile)
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(f"profile: {profile.name}")
        print(f"estimated_kbps: {summary['estimated_kbps']}")
        print(f"budget_margin_kbps: {summary['budget_margin_kbps']}")
        print("foxglove_allowlist:")
        for topic in profile.foxglove_allowlist:
            print(f"- {topic}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
