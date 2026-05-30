"""Tests for hardware-boundary fault scenario loading."""

from pathlib import Path

import pytest
from lunabot_bringup.hardware_fault_injection import (
    _message_from_config,
    load_scenarios,
    scenario_needs_live_topic_ack,
    validate_live_topic_ack,
)
from lunabot_bringup.message_fields import write_message_field
from rosidl_runtime_py.utilities import get_message


def _config_path() -> Path:
    return (
        Path(__file__).resolve().parents[1] / "config" / "hardware_fault_scenarios.yaml"
    )


def test_load_scenarios_includes_required_fault_cases():
    scenarios = load_scenarios(_config_path())

    assert set(scenarios) >= {
        "nominal",
        "drivetrain_controller_offline",
        "estop_asserted",
        "excavation_driver_fault",
        "localisation_lost",
    }


def test_scenarios_publish_only_absolute_topics():
    scenarios = load_scenarios(_config_path())

    for scenario in scenarios.values():
        for topic in scenario.topics:
            assert topic.name.startswith("/")


def test_all_configured_payloads_instantiate_as_ros_messages():
    scenarios = load_scenarios(_config_path())

    for scenario in scenarios.values():
        for topic in scenario.topics:
            _message_from_config(topic.type_name, topic.fields)


def test_safety_topic_scenarios_require_explicit_ack():
    scenario = load_scenarios(_config_path())["estop_asserted"]

    assert scenario_needs_live_topic_ack(scenario) is True
    with pytest.raises(ValueError, match="--allow-live-topics"):
        validate_live_topic_ack(scenario, allow_live_topics=False)


def test_non_safety_scenarios_do_not_require_live_ack():
    scenario = load_scenarios(_config_path())["drivetrain_controller_offline"]

    assert scenario_needs_live_topic_ack(scenario) is False
    validate_live_topic_ack(scenario, allow_live_topics=False)


def test_message_from_config_sets_status_fault_fields():
    message = _message_from_config(
        "lunabot_interfaces/msg/DrivetrainStatus",
        {
            "state": 4,
            "fault_code": 3,
            "controller_online": [False, False],
        },
    )

    assert message.state == 4
    assert message.fault_code == 3
    assert list(message.controller_online) == [False, False]


def test_set_field_supports_indexed_arrays():
    status_type = get_message("lunabot_interfaces/msg/DrivetrainStatus")
    message = status_type()

    write_message_field(message, "controller_online[0]", True)
    write_message_field(message, "controller_online[1]", False)

    assert list(message.controller_online) == [True, False]


def test_invalid_topic_rate_is_rejected(tmp_path):
    config = tmp_path / "faults.yaml"
    config.write_text(
        """
scenarios:
  bad:
    topics:
      - name: /drivetrain/status
        type: lunabot_interfaces/msg/DrivetrainStatus
        rate_hz: 0.0
        fields: {}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="rate_hz must be positive"):
        load_scenarios(config)
