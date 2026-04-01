"""Unit tests for phase-scoped preflight filtering."""

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
