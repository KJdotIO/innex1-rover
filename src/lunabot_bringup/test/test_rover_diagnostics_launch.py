"""Tests for the rover diagnostics launch file."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "rover_diagnostics.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "rover_diagnostics_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rover_diagnostics_launch_starts_movement_watchdog():
    module = _load_launch_module()
    description = module.generate_launch_description()

    declared_arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert declared_arguments == [
        "stale_timeout_s",
        "publish_hz",
        "movement_timeout_s",
        "movement_warn_s",
        "movement_auto_arm",
    ]
    assert [node.node_executable for node in nodes] == [
        "rover_diagnostics",
        "movement_watchdog",
    ]
