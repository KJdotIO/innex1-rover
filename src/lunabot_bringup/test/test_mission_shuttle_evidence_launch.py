import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "mission_shuttle_evidence.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "mission_shuttle_evidence_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mission_shuttle_evidence_launch_composes_one_cycle_stack():
    module = _load_launch_module()
    description = module.generate_launch_description()

    declared_arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    timers = [
        entity for entity in description.entities if isinstance(entity, TimerAction)
    ]

    assert declared_arguments == [
        "use_sim_time",
        "launch_rviz",
        "world",
        "waypoints_file",
        "max_shuttle_cycles",
        "disable_nav_gate",
        "enforce_preflight",
        "force_overcurrent",
        "force_driver_fault",
        "hold_home_switch_false",
    ]
    assert len(includes) == 3
    assert [node.node_executable for node in nodes] == [
        "estop_node",
        "material_action_server",
        "rover_diagnostics",
        "movement_watchdog",
    ]
    assert len(timers) == 1
    assert timers[0].period == 45.0
    assert [node.node_executable for node in timers[0].actions] == [
        "mission_manager",
    ]
