"""Unit tests for pure launch-gate helpers."""

from lunabot_bringup.launch_gate import select_preflight_config_path


def test_select_preflight_config_path_uses_default_in_normal_mode():
    result = select_preflight_config_path(
        False,
        "/tmp/preflight_checks.yaml",
        "/tmp/preflight_checks_lidar_debug.yaml",
    )

    assert result == "/tmp/preflight_checks.yaml"


def test_select_preflight_config_path_uses_lidar_debug_config_in_debug_mode():
    result = select_preflight_config_path(
        True,
        "/tmp/preflight_checks.yaml",
        "/tmp/preflight_checks_lidar_debug.yaml",
    )

    assert result == "/tmp/preflight_checks_lidar_debug.yaml"
