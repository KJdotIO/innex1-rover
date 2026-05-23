"""Unit tests for the top-level navigation launch wiring."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node


def _load_launch_module():
    """Import navigation.launch.py as a Python module."""
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "navigation.launch.py"
    )
    spec = importlib.util.spec_from_file_location("navigation_launch", launch_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _node_named(description, name: str) -> Node:
    """Return a node by configured launch name."""
    matches = [
        entity
        for entity in description.entities
        if isinstance(entity, Node) and entity.__dict__.get("_Node__node_name") == name
    ]
    assert len(matches) == 1
    return matches[0]


def _normalised_parameters(node: Node) -> dict:
    """Return node parameter dictionaries with launch substitutions resolved."""
    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = "true"
    params = {}
    for param_group in node.__dict__["_Node__parameters"]:
        for key, value in param_group.items():
            normalised_key = "".join(part.perform(context) for part in key)
            if isinstance(value, tuple):
                normalised_value = "".join(
                    part.perform(context) for part in value
                ).splitlines()[0]
            else:
                normalised_value = value
            params[normalised_key] = normalised_value
    return params


def test_navigation_launch_starts_wall_exclusion_filters(monkeypatch):
    """Raw point clouds are filtered before autonomy consumers start."""
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )

    description = module.generate_launch_description()

    for name, input_topic, output_topic in (
        (
            "arena_boundary_filter_camera_front",
            "/camera_front/points",
            "/perception/arena_boundary/camera_front/points",
        ),
        (
            "arena_boundary_filter_camera_rear",
            "/camera_rear/points",
            "/perception/arena_boundary/camera_rear/points",
        ),
        (
            "arena_boundary_filter_ouster",
            "/ouster/points",
            "/perception/arena_boundary/ouster/points",
        ),
    ):
        node = _node_named(description, name)
        params = _normalised_parameters(node)
        assert node.__dict__["_Node__package"] == "lunabot_perception"
        assert node.__dict__["_Node__node_executable"] == "arena_boundary_filter"
        assert params["input_topic"] == input_topic
        assert params["output_topic"] == output_topic
        assert params["target_frame"] == "odom"


def test_navigation_launch_feeds_crater_detection_from_filtered_front_cloud(
    monkeypatch,
):
    """Crater detection consumes the filtered front point cloud."""
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )

    description = module.generate_launch_description()
    crater_detection = _node_named(description, "crater_detection")

    assert crater_detection.__dict__["_Node__node_executable"] == "crater_detection"
    assert _normalised_parameters(crater_detection)["input_cloud_topic"] == (
        "/perception/arena_boundary/camera_front/points"
    )
