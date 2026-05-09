"""Unit tests for Jetson runtime profile guardrails."""

from pathlib import Path

from lunabot_bringup.runtime_profile import (
    COMPETITION_BUDGET_KBPS,
    COMPETITION_PROFILES,
    load_profiles,
    profile_summary,
    validate_profiles,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_PROFILES_PATH = PACKAGE_ROOT / "config" / "runtime_profiles.yaml"


def test_required_runtime_profiles_are_present():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    assert {
        "sim_debug",
        "sim_competition",
        "hardware_bringup",
        "hardware_competition",
    }.issubset(profiles)


def test_runtime_profiles_pass_guardrails():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    assert validate_profiles(profiles) == []


def test_competition_profiles_stay_below_rulebook_budget_with_margin():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    for name in COMPETITION_PROFILES:
        profile = profiles[name]
        assert profile.budget_kbps <= COMPETITION_BUDGET_KBPS
        assert profile.estimated_kbps <= profile.target_kbps
        assert profile.budget_margin_kbps >= 3000.0


def test_competition_foxglove_allowlists_exclude_raw_streams():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    for name in COMPETITION_PROFILES:
        exposed = " ".join(profiles[name].foxglove_allowlist).lower()
        assert "image" not in exposed
        assert "points" not in exposed
        assert "camera" not in exposed


def test_hardware_competition_keeps_debug_tools_off_by_default():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    off_by_default = " ".join(profiles["hardware_competition"].off_by_default).lower()

    assert "rviz2" in off_by_default
    assert "gazebo" in off_by_default
    assert "heavy rosbag" in off_by_default


def test_profile_summary_reports_estimated_bandwidth():
    profiles = load_profiles(RUNTIME_PROFILES_PATH)

    summary = profile_summary(profiles["hardware_competition"])

    assert summary["estimated_kbps"] > 0
    assert summary["budget_margin_kbps"] > 0
    assert summary["telemetry_topics"][0]["estimated_kbps"] > 0
