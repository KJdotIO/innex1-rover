"""Unit tests for localisation launch helper behaviour."""

from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _load_launch_module():
    """Import the localisation launch file as a Python module."""
    module_path = (
        Path(__file__).resolve().parents[1] / "launch" / "localisation.launch.py"
    )
    spec = spec_from_file_location("localisation_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("true", "True"),
        ("True", "True"),
        ("1", "True"),
        ("yes", "True"),
        ("on", "True"),
        ("false", "False"),
        ("False", "False"),
        ("0", "False"),
        ("no", "False"),
        ("off", "False"),
    ],
)
def test_is_truthy_accepts_common_launch_values(text, expected):
    launch_module = _load_launch_module()
    context = LaunchContext()

    expression = launch_module._is_truthy(text)

    assert expression.perform(context) == expected


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("false", "True"),
        ("False", "True"),
        ("0", "True"),
        ("no", "True"),
        ("off", "True"),
        ("true", "False"),
    ],
)
def test_is_falsey_accepts_common_launch_values(text, expected):
    launch_module = _load_launch_module()
    context = LaunchContext()

    expression = launch_module._is_falsey(text)

    assert expression.perform(context) == expected


@pytest.mark.parametrize(
    "text",
    ["maybe", "2", "", "treu"],
)
def test_normalise_bool_text_rejects_invalid_values(text):
    launch_module = _load_launch_module()

    with pytest.raises(ValueError):
        launch_module._normalise_bool_text(text)


def test_tag_pose_bridge_config_switches_between_sim_and_hardware_yaml():
    launch_module = _load_launch_module()
    context = LaunchContext()

    sim_result = launch_module._tag_pose_bridge_config(
        "True",
        "/tmp/tag_pose_bridge_sim.yaml",
        "/tmp/tag_pose_bridge.yaml",
    )
    hardware_result = launch_module._tag_pose_bridge_config(
        "0",
        "/tmp/tag_pose_bridge_sim.yaml",
        "/tmp/tag_pose_bridge.yaml",
    )

    assert sim_result.perform(context) == "/tmp/tag_pose_bridge_sim.yaml"
    assert hardware_result.perform(context) == "/tmp/tag_pose_bridge.yaml"


def test_validate_boolean_launch_arguments_rejects_invalid_launch_value():
    launch_module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["lidar_costmap_phase"] = "treu"
    context.launch_configurations["enable_visual_slam"] = "false"
    context.launch_configurations["use_sim_time"] = "true"
    context.launch_configurations["enable_apriltag_debug"] = "false"

    with pytest.raises(ValueError, match="lidar_costmap_phase"):
        launch_module._validate_boolean_launch_arguments(context)


def test_generate_launch_description_has_validation_and_one_tag_pose_node():
    launch_module = _load_launch_module()
    launch_module.get_package_share_directory = (
        lambda _package: "/tmp/lunabot_localisation"
    )
    description = launch_module.generate_launch_description()

    validators = [
        entity
        for entity in description.entities
        if isinstance(entity, OpaqueFunction)
    ]
    tag_pose_publishers = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.__dict__.get("_Node__node_name") == "tag_pose_publisher"
    ]

    assert len(validators) == 1
    assert len(tag_pose_publishers) == 1
