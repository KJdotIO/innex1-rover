"""Unit tests for phase-scoped preflight filtering and CLI helpers."""

import sys

import pytest

from lunabot_bringup.preflight_check import _parse_bool_text
from lunabot_bringup.preflight_check import _strip_ros_cli_args
from lunabot_bringup.preflight_profiles import filter_preflight_config
from lunabot_bringup.preflight_profiles import validate_phase


def test_filter_preflight_config_keeps_all_checks_in_full_phase():
    config = {
        "preflight": {
            "required_topics": [
                {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu"},
                {
                    "name": "/navigate_to_pose",
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
                {"name": "/navigate_to_pose", "phases": ["runtime"]},
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
