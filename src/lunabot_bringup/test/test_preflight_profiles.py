"""Unit tests for phase-scoped preflight filtering and CLI helpers."""

import sys
from pathlib import Path

import pytest
import yaml

from lunabot_bringup.preflight_check import _parse_bool_text
from lunabot_bringup.preflight_check import _strip_ros_cli_args
from lunabot_bringup.preflight_profiles import filter_preflight_config
from lunabot_bringup.preflight_profiles import validate_phase


def test_filter_preflight_config_keeps_all_checks_in_full_phase():
    config = {
        "preflight": {
            "required_topics": [
                {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu"},
            ],
            "required_actions": [
                {
                    "name": "/navigate_to_pose",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
                {
                    "name": "/navigate_to_pose_gate",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
            ],
            "required_nodes": [
                {"name": "map_server"},
                {"name": "bt_navigator", "phases": ["runtime"]},
            ],
        }
    }

    filtered = filter_preflight_config(config, "full")

    assert filtered == config


def test_filter_preflight_config_drops_runtime_only_checks_from_launch_phase():
    config = {
        "preflight": {
            "required_actions": [
                {
                    "name": "/navigate_to_pose",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
                {
                    "name": "/navigate_to_pose_gate",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
            ],
            "required_nodes": [
                {"name": "map_server"},
                {"name": "bt_navigator", "phases": ["runtime"]},
                {"name": "apriltag", "phases": ["launch", "runtime"]},
            ],
        }
    }

    filtered = filter_preflight_config(config, "launch")

    assert filtered["preflight"]["required_actions"] == []
    assert filtered["preflight"]["required_nodes"] == [
        {"name": "map_server"},
        {"name": "apriltag", "phases": ["launch", "runtime"]},
    ]


def test_filter_preflight_config_keeps_runtime_actions_in_runtime_phase():
    config = {
        "preflight": {
            "required_actions": [
                {
                    "name": "/navigate_to_pose",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
                {
                    "name": "/navigate_to_pose_gate",
                    "type": "nav2_msgs/action/NavigateToPose",
                    "phases": ["runtime"],
                },
            ],
        }
    }

    filtered = filter_preflight_config(config, "runtime")

    assert filtered["preflight"]["required_actions"] == config["preflight"][
        "required_actions"
    ]


def test_filter_preflight_config_rejects_invalid_phase_shape():
    config = {
        "preflight": {
            "required_nodes": [
                {"name": "bt_navigator", "phases": "runtime"},
            ]
        }
    }

    try:
        filter_preflight_config(config, "launch")
    except ValueError as exc:
        assert "expected a list" in str(exc)
    else:
        raise AssertionError("Expected invalid phases value to raise ValueError")


def test_validate_phase_rejects_unknown_phase_name():
    try:
        validate_phase("launc")
    except ValueError as exc:
        assert "Unsupported preflight phase" in str(exc)
    else:
        raise AssertionError("Expected invalid phase name to raise ValueError")


def test_strip_ros_cli_args_keeps_only_executable_flags():
    cli_args = _strip_ros_cli_args(
        [
            "--phase",
            "launch",
            "--use-sim-time",
            "false",
            "--ros-args",
            "-r",
            "__node:=preflight_gate",
        ]
    )

    assert cli_args == ["--phase", "launch", "--use-sim-time", "false"]


def test_strip_ros_cli_args_uses_process_argv_when_args_omitted(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "/tmp/preflight_check",
            "--phase",
            "launch",
            "--ros-args",
            "-r",
            "__node:=preflight_gate",
        ],
    )

    assert _strip_ros_cli_args(None) == ["--phase", "launch"]


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("true", True),
        ("True", True),
        ("1", True),
        ("false", False),
        ("False", False),
        ("0", False),
    ],
)
def test_parse_bool_text_accepts_common_launch_values(text, expected):
    assert _parse_bool_text(text) is expected


def test_parse_bool_text_rejects_invalid_value():
    with pytest.raises(Exception):
        _parse_bool_text("maybe")


def test_full_preflight_profile_requires_terrain_hazard_pipeline():
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "preflight_checks.yaml"
    )

    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    required_topics = {
        topic["name"]: topic for topic in config["preflight"]["required_topics"]
    }
    required_nodes = {
        node["name"]: node for node in config["preflight"]["required_nodes"]
    }

    assert required_topics["/camera_front/points"]["critical"] is True
    assert required_topics["/terrain_hazard/grid"]["critical"] is True
    assert required_nodes["terrain_hazard_detector"]["critical"] is True


def test_lidar_debug_profile_keeps_terrain_hazard_optional():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "preflight_checks_lidar_debug.yaml"
    )

    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    required_topics = {
        topic["name"]: topic for topic in config["preflight"]["required_topics"]
    }
    required_nodes = {
        node["name"]: node for node in config["preflight"]["required_nodes"]
    }

    assert required_topics["/camera_front/points"]["critical"] is False
    assert required_topics["/terrain_hazard/grid"]["critical"] is False
    assert required_nodes["terrain_hazard_detector"]["critical"] is False
