"""Unit tests for phase-scoped preflight filtering and CLI helpers."""

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from lunabot_bringup.preflight_check import (
    ACTION_TYPE_MAP,
    DURABILITY_MAP,
    RELIABILITY_MAP,
    _merge_contract_requirements,
    _parse_bool_text,
    _parse_qos_policy,
    _strip_ros_cli_args,
)
from lunabot_bringup.preflight_profiles import filter_preflight_config, validate_phase


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


def test_dry_run_preflight_config_keeps_mission_actions_in_runtime_phase():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "preflight_checks_dry_run.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    filtered = filter_preflight_config(config, "runtime")
    action_names = {
        entry["name"] for entry in filtered["preflight"]["required_actions"]
    }

    assert "/navigate_to_pose_gate" in action_names
    assert "/mission/excavate" in action_names
    assert "/mission/deposit" in action_names


def test_action_type_map_supports_mission_dry_run_actions():
    assert "lunabot_interfaces/action/Excavate" in ACTION_TYPE_MAP
    assert "lunabot_interfaces/action/Deposit" in ACTION_TYPE_MAP


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
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_bool_text("maybe")


def test_parse_qos_policy_rejects_unknown_value():
    with pytest.raises(ValueError, match="Unsupported reliability policy"):
        _parse_qos_policy(
            "eventual",
            RELIABILITY_MAP,
            field_name="reliability",
        )


def test_parse_qos_policy_accepts_known_value():
    assert (
        _parse_qos_policy(
            "transient_local",
            DURABILITY_MAP,
            field_name="durability",
        )
        == DURABILITY_MAP["transient_local"]
    )


def test_parse_qos_policy_rejects_unknown_durability_value():
    with pytest.raises(ValueError, match="Unsupported durability policy"):
        _parse_qos_policy(
            "persistent",
            DURABILITY_MAP,
            field_name="durability",
        )


def test_merge_contract_requirements_ignores_malformed_existing_entries(
    tmp_path, monkeypatch
):
    contract_path = tmp_path / "contracts" / "interfaces.json"
    contract_path.parent.mkdir(parents=True)
    contract_path.write_text(
        json.dumps(
            {
                "topics": [
                    {
                        "kind": "publisher",
                        "name": "/imu/data_raw",
                        "type": "sensor_msgs/msg/Imu",
                    }
                ],
                "tf_links": [
                    {
                        "parent": "map",
                        "child": "base_link",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    config = {
        "preflight": {
            "required_topics": ["bad-topic-entry"],
            "required_tf_links": ["bad-tf-entry"],
        }
    }
    logger = SimpleNamespace(info=lambda _msg: None, warning=lambda _msg: None)

    monkeypatch.setattr(
        "lunabot_bringup.preflight_check._repo_contract_path",
        lambda: contract_path,
    )

    merged = _merge_contract_requirements(config, logger)

    assert merged["preflight"]["required_topics"][1]["name"] == "/imu/data_raw"
    assert merged["preflight"]["required_tf_links"][1] == {
        "parent": "map",
        "child": "base_link",
        "critical": True,
        "phases": None,
    }
