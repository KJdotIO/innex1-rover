import importlib.util
from pathlib import Path

from launch import LaunchContext
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


def test_mission_shuttle_evidence_launch_composes_one_cycle_stack(monkeypatch):
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/tmp/{package}",
    )
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
        "ouster_horizontal_samples",
        "ouster_vertical_samples",
        "waypoints_file",
        "max_shuttle_cycles",
        "mission_start_delay_s",
        "nav_goal_timeout_s",
        "lidar_costmap_phase",
        "lidar_odometry_backend",
        "nav_params_file",
        "collision_monitor_params_file",
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
    context = LaunchContext()
    context.launch_configurations["mission_start_delay_s"] = "75.0"
    assert timers[0].period[0].perform(context) == "75.0"
    assert [node.node_executable for node in timers[0].actions] == [
        "mission_manager",
    ]
