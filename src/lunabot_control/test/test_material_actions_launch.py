"""Tests for the standalone material bench launch."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "material_actions.launch.py"
    spec = importlib.util.spec_from_file_location("material_actions_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_material_actions_launch_includes_bench_mock_and_servers():
    module = _load_launch_module()
    launch_source = (
        Path(__file__).resolve().parents[1] / "launch" / "material_actions.launch.py"
    ).read_text()

    description = module.generate_launch_description()
    entities = [entity for entity in description.entities if isinstance(entity, Node)]
    node_names = {entity.__dict__.get("_Node__node_name") for entity in entities}

    assert node_names == {
        "excavation_telemetry_mock",
        "excavation_controller",
        "excavation_action_server",
        "material_action_server",
    }

    mock_node = next(
        entity
        for entity in entities
        if entity.__dict__.get("_Node__node_name") == "excavation_telemetry_mock"
    )
    assert type(mock_node.condition).__name__ == "IfCondition"
    assert str(mock_node.condition._predicate_expression[0]) == "use_mock_telemetry"
    assert '"fault_on_start_code": LaunchConfiguration("fault_on_start_code")' in launch_source
    assert '"fault_on_stop_code": LaunchConfiguration("fault_on_stop_code")' in launch_source

    declared_arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert declared_arguments == [
        "fault_on_start_code",
        "use_mock_telemetry",
        "fault_on_stop_code",
    ]
