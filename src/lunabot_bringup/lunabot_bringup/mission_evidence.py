"""Create repeatable rosbag evidence packs for mission dry runs."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

DEFAULT_OUTPUT_ROOT = Path("~/innex1_mission_evidence").expanduser()
DEFAULT_STORAGE = "sqlite3"
DEFAULT_MAX_BAG_DURATION_S = 300
DEFAULT_MAX_BAG_SIZE_BYTES = 2_000_000_000
DEFAULT_PROFILE = "minimal"
DEFAULT_SNAPSHOT_RELATIVE_PATHS = (
    "config/arena_waypoints.yaml",
    "config/preflight_checks_dry_run.yaml",
    "config/rosbag_evidence_profiles.yaml",
    "launch/mission_shuttle_evidence.launch.py",
)
SUMMARY_STEPS = ("travel", "excavate", "deposit", "overall")
MISSION_HALTED_RE = re.compile(r"Mission halted after (?P<cycles>\d+) cycles?\.")


@dataclass(frozen=True)
class EvidenceProfile:
    """Topic allowlist for one evidence recording mode."""

    name: str
    description: str
    topics: tuple[str, ...]


@dataclass(frozen=True)
class EvidencePlan:
    """Resolved paths and commands for one evidence pack."""

    pack_dir: Path
    bag_dir: Path
    logs_dir: Path
    config_dir: Path
    manifest_path: Path
    record_command: tuple[str, ...]
    replay_command: tuple[str, ...]
    mission_command: tuple[str, ...]
    profile: EvidenceProfile


def _utc_stamp() -> str:
    """Return a filesystem-safe UTC timestamp."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _slug(value: str) -> str:
    """Return a conservative folder-name fragment."""
    cleaned = "".join(
        char.lower() if char.isalnum() else "-" for char in value.strip()
    ).strip("-")
    while "--" in cleaned:
        cleaned = cleaned.replace("--", "-")
    return cleaned or "mission"


def default_profiles_path() -> Path:
    """Return the installed evidence profile config path."""
    from ament_index_python.packages import get_package_share_directory

    share_dir = Path(get_package_share_directory("lunabot_bringup"))
    return share_dir / "config" / "rosbag_evidence_profiles.yaml"


def load_profiles(path: Path) -> dict[str, EvidenceProfile]:
    """Load evidence profiles from YAML."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("profiles"), dict):
        raise ValueError(f"invalid evidence profile file: {path}")

    profiles: dict[str, EvidenceProfile] = {}
    for name, raw_profile in data["profiles"].items():
        if not isinstance(name, str) or not isinstance(raw_profile, dict):
            raise ValueError(f"invalid evidence profile entry in {path}")

        raw_topics = raw_profile.get("topics")
        if not isinstance(raw_topics, list) or not raw_topics:
            raise ValueError(f"profile {name!r} must list at least one topic")

        topics = tuple(str(topic) for topic in raw_topics)
        if any(not topic.startswith("/") for topic in topics):
            raise ValueError(f"profile {name!r} contains a non-absolute topic")

        profiles[name] = EvidenceProfile(
            name=name,
            description=str(raw_profile.get("description", "")),
            topics=topics,
        )
    return profiles


def git_sha(cwd: Path | None = None) -> str:
    """Return the current git SHA, or `unknown` outside a checkout."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return result.stdout.strip() or "unknown"


def build_record_command(
    profile: EvidenceProfile,
    bag_dir: Path,
    *,
    storage: str = DEFAULT_STORAGE,
    max_bag_duration_s: int = DEFAULT_MAX_BAG_DURATION_S,
    max_bag_size_bytes: int = DEFAULT_MAX_BAG_SIZE_BYTES,
    use_sim_time: bool = False,
) -> tuple[str, ...]:
    """Build the rosbag2 record command for one evidence profile."""
    command = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(bag_dir),
        "-s",
        storage,
        "--max-bag-duration",
        str(max_bag_duration_s),
        "--max-bag-size",
        str(max_bag_size_bytes),
    ]
    if use_sim_time:
        command.append("--use-sim-time")
    command.extend(profile.topics)
    return tuple(command)


def create_evidence_plan(
    *,
    profile: EvidenceProfile,
    output_root: Path,
    label: str,
    storage: str = DEFAULT_STORAGE,
    max_bag_duration_s: int = DEFAULT_MAX_BAG_DURATION_S,
    max_bag_size_bytes: int = DEFAULT_MAX_BAG_SIZE_BYTES,
    use_sim_time: bool = False,
    mission_command: Sequence[str] = (),
    timestamp: str | None = None,
) -> EvidencePlan:
    """Resolve the folder layout and commands for one evidence pack."""
    stamp = timestamp or _utc_stamp()
    pack_dir = output_root.expanduser() / f"{stamp}_{_slug(label)}"
    bag_dir = pack_dir / "bag"
    logs_dir = pack_dir / "logs"
    config_dir = pack_dir / "config"
    record_command = build_record_command(
        profile,
        bag_dir,
        storage=storage,
        max_bag_duration_s=max_bag_duration_s,
        max_bag_size_bytes=max_bag_size_bytes,
        use_sim_time=use_sim_time,
    )
    return EvidencePlan(
        pack_dir=pack_dir,
        bag_dir=bag_dir,
        logs_dir=logs_dir,
        config_dir=config_dir,
        manifest_path=pack_dir / "manifest.json",
        record_command=record_command,
        replay_command=("ros2", "bag", "play", str(bag_dir)),
        mission_command=tuple(mission_command),
        profile=profile,
    )


def initialise_pack(plan: EvidencePlan) -> None:
    """Create the evidence-pack directory layout."""
    plan.logs_dir.mkdir(parents=True, exist_ok=False)
    plan.config_dir.mkdir(parents=True, exist_ok=False)


def snapshot_configs(
    *,
    plan: EvidencePlan,
    package_share: Path,
    relative_paths: Sequence[str] = DEFAULT_SNAPSHOT_RELATIVE_PATHS,
) -> list[str]:
    """Copy useful run configs into the evidence pack."""
    copied: list[str] = []
    for relative_path in relative_paths:
        source = package_share / relative_path
        if not source.exists():
            continue

        destination = plan.config_dir / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied.append(relative_path)
    return copied


def write_manifest(
    *,
    plan: EvidencePlan,
    git_revision: str,
    config_snapshots: Sequence[str],
    started_at_utc: str,
    completed_at_utc: str | None = None,
    mission_return_code: int | None = None,
    recording_return_code: int | None = None,
    mission_summary: dict[str, Any] | None = None,
) -> None:
    """Write a machine-readable summary for the evidence pack."""
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "started_at_utc": started_at_utc,
        "completed_at_utc": completed_at_utc,
        "git_sha": git_revision,
        "profile": {
            "name": plan.profile.name,
            "description": plan.profile.description,
            "topics": list(plan.profile.topics),
        },
        "paths": {
            "bag": str(plan.bag_dir),
            "logs": str(plan.logs_dir),
            "config": str(plan.config_dir),
        },
        "commands": {
            "record": list(plan.record_command),
            "replay": list(plan.replay_command),
            "mission": list(plan.mission_command),
        },
        "config_snapshots": list(config_snapshots),
        "results": {
            "mission_return_code": mission_return_code,
            "recording_return_code": recording_return_code,
            "mission_summary": mission_summary or {},
        },
    }
    plan.manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def parse_mission_summary(log_path: Path) -> dict[str, Any]:
    """Extract readable mission result details from a command log."""
    if not log_path.exists():
        return {}

    step_results: dict[str, str] = {}
    completed_cycles: int | None = None
    safe_fail = False
    failure_reason = ""
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        halted_match = MISSION_HALTED_RE.search(line)
        if halted_match is not None:
            completed_cycles = int(halted_match.group("cycles"))

        if "Safe-fail triggered" in line:
            safe_fail = True
            failure_reason = "safe-fail triggered"

        if "Safety stop during mission" in line:
            safe_fail = True
            failure_reason = "safety stop during mission"

        if ": " in line:
            name, value = line.split(": ", 1)
            if name in SUMMARY_STEPS and value in {"pass", "fail"}:
                step_results[name] = value

    if completed_cycles is not None:
        overall = "fail" if safe_fail or completed_cycles == 0 else "pass"
        return {
            "completed_cycles": completed_cycles,
            "overall": overall,
            "failure_reason": failure_reason,
        }

    if not step_results:
        return {}

    failed_steps = [
        step
        for step in SUMMARY_STEPS
        if step != "overall" and step_results.get(step) == "fail"
    ]
    if step_results.get("overall") == "fail":
        if failed_steps:
            failure_reason = f"{failed_steps[0]} failed"
        else:
            failure_reason = "mission failed"

    return {
        "step_results": step_results,
        "overall": step_results.get("overall", "unknown"),
        "failure_reason": failure_reason,
    }


def _terminate(process: subprocess.Popen[Any]) -> int:
    """Terminate one process group and return the final code."""
    if process.poll() is not None:
        return int(process.returncode)

    os.killpg(process.pid, signal.SIGINT)
    try:
        return int(process.wait(timeout=10.0))
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return int(process.wait(timeout=5.0))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return int(process.wait(timeout=5.0))


def run_recording(plan: EvidencePlan) -> tuple[int | None, int]:
    """Run rosbag recording, optionally around a mission command."""
    record_log = (plan.logs_dir / "rosbag_record.log").open(
        "w",
        encoding="utf-8",
    )
    try:
        recorder = subprocess.Popen(
            plan.record_command,
            stdout=record_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(2.0)

        mission_return_code: int | None = None
        if plan.mission_command:
            mission_log = mission_log_path(plan).open(
                "w",
                encoding="utf-8",
            )
            try:
                mission = subprocess.run(
                    plan.mission_command,
                    stdout=mission_log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
                mission_return_code = int(mission.returncode)
            finally:
                mission_log.close()
        else:
            recorder.wait()

        recording_return_code = _terminate(recorder)
        return mission_return_code, recording_return_code
    finally:
        record_log.close()


def mission_log_path(plan: EvidencePlan) -> Path:
    """Return the standard mission command log path."""
    return plan.logs_dir / "mission_command.log"


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a rosbag-backed mission evidence pack.",
    )
    parser.add_argument("mission_command", nargs=argparse.REMAINDER)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--profiles-file", type=Path, default=None)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--label", default="dry-run")
    parser.add_argument("--storage", default=DEFAULT_STORAGE)
    parser.add_argument(
        "--max-bag-duration-s",
        type=int,
        default=DEFAULT_MAX_BAG_DURATION_S,
    )
    parser.add_argument(
        "--max-bag-size-bytes",
        type=int,
        default=DEFAULT_MAX_BAG_SIZE_BYTES,
    )
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Create the evidence pack and manifest without starting rosbag.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Create and optionally record a mission evidence pack."""
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    profiles_path = args.profiles_file or default_profiles_path()
    profiles = load_profiles(profiles_path)
    if args.profile not in profiles:
        print(f"unknown evidence profile: {args.profile}", file=sys.stderr)
        return 2

    mission_command = tuple(args.mission_command)
    if mission_command and mission_command[0] == "--":
        mission_command = mission_command[1:]

    plan = create_evidence_plan(
        profile=profiles[args.profile],
        output_root=args.output_root,
        label=args.label,
        storage=args.storage,
        max_bag_duration_s=args.max_bag_duration_s,
        max_bag_size_bytes=args.max_bag_size_bytes,
        use_sim_time=args.use_sim_time,
        mission_command=mission_command,
    )
    initialise_pack(plan)

    package_share = profiles_path.parent.parent
    config_snapshots = snapshot_configs(
        plan=plan,
        package_share=package_share,
    )
    started_at_utc = datetime.now(timezone.utc).isoformat()
    write_manifest(
        plan=plan,
        git_revision=git_sha(Path.cwd()),
        config_snapshots=config_snapshots,
        started_at_utc=started_at_utc,
    )

    print(f"evidence_pack: {plan.pack_dir}")
    print("record_command:", " ".join(plan.record_command))
    print("replay_command:", " ".join(plan.replay_command))

    if args.plan_only:
        return 0

    mission_code, recording_code = run_recording(plan)
    write_manifest(
        plan=plan,
        git_revision=git_sha(Path.cwd()),
        config_snapshots=config_snapshots,
        started_at_utc=started_at_utc,
        completed_at_utc=datetime.now(timezone.utc).isoformat(),
        mission_return_code=mission_code,
        recording_return_code=recording_code,
        mission_summary=parse_mission_summary(mission_log_path(plan)),
    )
    if mission_code not in (None, 0):
        return mission_code
    return 0 if recording_code in (0, -2, 130) else recording_code


if __name__ == "__main__":
    raise SystemExit(main())
