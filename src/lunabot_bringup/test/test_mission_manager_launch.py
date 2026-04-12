import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "mission_manager.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "mission_manager_launch", launch_path
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mission_manager_launch_starts_one_supervisor_node(monkeypatch):
    module = _load_launch_module()
    description = module.generate_launch_description()

    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    declared_arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert declared_arguments == ["use_sim_time"]
    assert len(nodes) == 1
    assert nodes[0].node_executable == "mission_manager"

    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = "false"
    monkeypatch.setattr(ExecuteProcess, "execute", lambda _self, _context: None)
    nodes[0].execute(context)

    assert nodes[0].node_name.rsplit("/", 1)[-1] == "mission_manager"
