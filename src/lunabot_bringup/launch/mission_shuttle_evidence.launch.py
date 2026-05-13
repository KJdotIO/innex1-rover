"""Launch one waypoint-based mission cycle for evidence recording."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_file(package_name: str, *parts: str) -> str:
    """Return one launch file path from a package share directory."""
    return str(Path(get_package_share_directory(package_name)).joinpath(*parts))


def _handle_mission_exit(event, _context):
    """Shut the evidence stack down when the mission manager exits."""
    if event.returncode == 0:
        return [
            LogInfo(msg="Mission shuttle evidence run complete; shutting down."),
            EmitEvent(event=Shutdown(reason="mission shuttle evidence complete")),
        ]

    return [
        LogInfo(
            msg=(
                "Mission shuttle evidence run exited with a non-zero code; "
                "shutting down."
            )
        ),
        EmitEvent(event=Shutdown(reason="mission shuttle evidence failed")),
    ]


def generate_launch_description():
    """Compose sim, navigation, mechanism mocks and one mission-manager cycle."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = LaunchConfiguration("world")
    waypoints_file = LaunchConfiguration("waypoints_file")
    max_shuttle_cycles = LaunchConfiguration("max_shuttle_cycles")
    disable_nav_gate = LaunchConfiguration("disable_nav_gate")
    enforce_preflight = LaunchConfiguration("enforce_preflight")
    force_overcurrent = LaunchConfiguration("force_overcurrent")
    force_driver_fault = LaunchConfiguration("force_driver_fault")
    hold_home_switch_false = LaunchConfiguration("hold_home_switch_false")

    moon_yard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_simulation", "launch", "moon_yard.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_bringup", "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "enforce_preflight": enforce_preflight,
            "disable_nav_gate": disable_nav_gate,
            "camera_info_topic": "/camera_front/camera_info_synced",
            "sync_sim_camera_info": "true",
        }.items(),
    )

    excavation_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_excavation", "launch", "excavation_sim.launch.py")
        ),
        launch_arguments={
            "force_overcurrent": force_overcurrent,
            "force_driver_fault": force_driver_fault,
            "hold_home_switch_false": hold_home_switch_false,
        }.items(),
    )

    estop_node = Node(
        package="lunabot_safety",
        executable="estop_node",
        name="estop_node",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    deposit_action_server = Node(
        package="lunabot_control",
        executable="material_action_server",
        name="material_action_server",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    rover_diagnostics = Node(
        package="lunabot_bringup",
        executable="rover_diagnostics",
        name="rover_diagnostics",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    mission_manager = Node(
        package="lunabot_bringup",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            waypoints_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "max_shuttle_cycles": ParameterValue(
                    max_shuttle_cycles,
                    value_type=int,
                ),
            },
        ],
    )

    mission_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_manager,
            on_exit=_handle_mission_exit,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("world", default_value="moon_yard_craters"),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=_launch_file(
                    "lunabot_bringup",
                    "config",
                    "arena_waypoints.yaml",
                ),
            ),
            DeclareLaunchArgument("max_shuttle_cycles", default_value="1"),
            DeclareLaunchArgument("disable_nav_gate", default_value="true"),
            DeclareLaunchArgument("enforce_preflight", default_value="false"),
            DeclareLaunchArgument("force_overcurrent", default_value="false"),
            DeclareLaunchArgument("force_driver_fault", default_value="false"),
            DeclareLaunchArgument("hold_home_switch_false", default_value="false"),
            moon_yard,
            navigation,
            excavation_sim,
            estop_node,
            deposit_action_server,
            rover_diagnostics,
            TimerAction(period=45.0, actions=[mission_manager]),
            mission_exit_handler,
        ]
    )
