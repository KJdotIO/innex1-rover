"""Tests for the standalone mission manager launch entrypoint."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _load_launch_module():
    """Import the mission manager launch file as a Python module."""
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "mission_manager.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "mission_manager_launch", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mission_manager_launch_starts_one_supervisor_node():
    """The launch entrypoint should expose a single mission_manager node."""
    module = _load_launch_module()
    launch_source = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "mission_manager.launch.py"
    ).read_text()
    description = module.generate_launch_description()

    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    declared_arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert declared_arguments == ["use_sim_time"]
    assert len(nodes) == 1
    assert nodes[0].__dict__.get("_Node__node_name") == "mission_manager"
    assert 'executable="mission_manager"' in launch_source
