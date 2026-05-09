"""Unit tests for mission evidence pack helpers."""

import json
from pathlib import Path

import pytest

from lunabot_bringup.mission_evidence import (
    DEFAULT_MAX_BAG_DURATION_S,
    EvidenceProfile,
    build_record_command,
    create_evidence_plan,
    load_profiles,
    parse_mission_summary,
    snapshot_configs,
    write_manifest,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_load_profiles_rejects_non_absolute_topics(tmp_path):
    profiles_path = tmp_path / "profiles.yaml"
    profiles_path.write_text(
        "profiles:\n"
        "  bad:\n"
        "    description: bad\n"
        "    topics:\n"
        "      - relative_topic\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="non-absolute"):
        load_profiles(profiles_path)


def test_load_profiles_reads_minimal_topic_allowlist():
    profiles = load_profiles(PACKAGE_ROOT / "config/rosbag_evidence_profiles.yaml")

    minimal = profiles["minimal"]

    assert "/mission/state" in minimal.topics
    assert "/diagnostics" in minimal.topics
    assert "/ouster/points" not in minimal.topics
    assert profiles["heavy"].topics[-1] == "/ouster/points"


def test_build_record_command_uses_splitting_and_sim_time():
    profile = EvidenceProfile(
        name="minimal",
        description="test",
        topics=("/clock", "/mission/state"),
    )

    command = build_record_command(
        profile,
        Path("/tmp/pack/bag"),
        max_bag_duration_s=60,
        max_bag_size_bytes=1000,
        use_sim_time=True,
    )

    assert command[:5] == ("ros2", "bag", "record", "-o", "/tmp/pack/bag")
    assert "--max-bag-duration" in command
    assert "60" in command
    assert "--max-bag-size" in command
    assert "1000" in command
    assert "--use-sim-time" in command
    assert command[-2:] == ("/clock", "/mission/state")


def test_create_evidence_plan_uses_stable_layout(tmp_path):
    profile = EvidenceProfile(
        name="minimal",
        description="test",
        topics=("/clock",),
    )

    plan = create_evidence_plan(
        profile=profile,
        output_root=tmp_path,
        label="Good Dry Run!",
        timestamp="20260508T120000Z",
    )

    assert plan.pack_dir == tmp_path / "20260508T120000Z_good-dry-run"
    assert plan.bag_dir == plan.pack_dir / "bag"
    assert plan.logs_dir == plan.pack_dir / "logs"
    assert plan.config_dir == plan.pack_dir / "config"
    assert str(DEFAULT_MAX_BAG_DURATION_S) in plan.record_command
    assert plan.replay_command == ("ros2", "bag", "play", str(plan.bag_dir))


def test_snapshot_configs_copies_existing_files_only(tmp_path):
    package_share = tmp_path / "share"
    source = package_share / "config" / "arena_waypoints.yaml"
    source.parent.mkdir(parents=True)
    source.write_text("waypoints: {}\n", encoding="utf-8")
    profile = EvidenceProfile("minimal", "test", ("/clock",))
    plan = create_evidence_plan(
        profile=profile,
        output_root=tmp_path / "out",
        label="run",
        timestamp="20260508T120000Z",
    )
    plan.config_dir.mkdir(parents=True)

    copied = snapshot_configs(
        plan=plan,
        package_share=package_share,
        relative_paths=("config/arena_waypoints.yaml", "config/missing.yaml"),
    )

    assert copied == ["config/arena_waypoints.yaml"]
    assert (plan.config_dir / "config" / "arena_waypoints.yaml").exists()


def test_write_manifest_records_commands_and_results(tmp_path):
    profile = EvidenceProfile("minimal", "test", ("/clock",))
    plan = create_evidence_plan(
        profile=profile,
        output_root=tmp_path,
        label="run",
        mission_command=("ros2", "launch", "lunabot_bringup", "mission.py"),
        timestamp="20260508T120000Z",
    )
    plan.pack_dir.mkdir(parents=True)

    write_manifest(
        plan=plan,
        git_revision="abc123",
        config_snapshots=("config/arena_waypoints.yaml",),
        started_at_utc="2026-05-08T12:00:00+00:00",
        completed_at_utc="2026-05-08T12:01:00+00:00",
        mission_return_code=0,
        recording_return_code=130,
        mission_summary={"overall": "pass"},
    )

    manifest = json.loads(plan.manifest_path.read_text(encoding="utf-8"))
    assert manifest["git_sha"] == "abc123"
    assert manifest["profile"]["topics"] == ["/clock"]
    assert manifest["commands"]["replay"] == [
        "ros2",
        "bag",
        "play",
        str(plan.bag_dir),
    ]
    assert manifest["results"] == {
        "mission_return_code": 0,
        "mission_summary": {"overall": "pass"},
        "recording_return_code": 130,
    }


def test_parse_mission_summary_extracts_flat_dry_run_results(tmp_path):
    log_path = tmp_path / "mission_command.log"
    log_path.write_text(
        "noise before\ntravel: pass\nexcavate: fail\ndeposit: fail\noverall: fail\n",
        encoding="utf-8",
    )

    summary = parse_mission_summary(log_path)

    assert summary == {
        "step_results": {
            "travel": "pass",
            "excavate": "fail",
            "deposit": "fail",
            "overall": "fail",
        },
        "overall": "fail",
        "failure_reason": "excavate failed",
    }


def test_parse_mission_summary_extracts_shuttle_cycle_count(tmp_path):
    log_path = tmp_path / "mission_command.log"
    log_path.write_text(
        "[mission_manager]: Completed cycle 1/1\n"
        "[mission_manager]: Mission halted after 1 cycles.\n",
        encoding="utf-8",
    )

    summary = parse_mission_summary(log_path)

    assert summary == {
        "completed_cycles": 1,
        "overall": "pass",
        "failure_reason": "",
    }


def test_parse_mission_summary_marks_safe_fail_as_failure(tmp_path):
    log_path = tmp_path / "mission_command.log"
    log_path.write_text(
        "[mission_manager]: Safe-fail triggered; halting mission.\n"
        "[mission_manager]: Mission halted after 0 cycles.\n",
        encoding="utf-8",
    )

    summary = parse_mission_summary(log_path)

    assert summary == {
        "completed_cycles": 0,
        "overall": "fail",
        "failure_reason": "safe-fail triggered",
    }
