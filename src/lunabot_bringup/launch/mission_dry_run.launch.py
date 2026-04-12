"""Launch the full simulation stack and run one mission dry run."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
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


def _handle_harness_exit(event, _context):
    """Shut the stack down or fail the launch from the harness exit code."""
    if event.returncode == 0:
        return [
            LogInfo(msg="Mission dry-run harness passed; shutting down."),
            EmitEvent(event=Shutdown(reason="mission dry run complete")),
        ]

    return [
        LogInfo(msg="Mission dry-run harness failed; aborting launch."),
        OpaqueFunction(function=_raise_harness_failure),
    ]


def _raise_harness_failure(_context):
    """Abort launch with a non-zero exit when the harness fails."""
    raise RuntimeError("Mission dry-run harness failed")


def generate_launch_description():
    """Compose the simulation stack and the one-shot mission harness."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    runtime_preflight_enabled = LaunchConfiguration("runtime_preflight_enabled")
    preflight_config = LaunchConfiguration("preflight_config")
    force_overcurrent = LaunchConfiguration("force_overcurrent")
    force_driver_fault = LaunchConfiguration("force_driver_fault")
    hold_home_switch_false = LaunchConfiguration("hold_home_switch_false")
    travel_frame_id = LaunchConfiguration("travel_frame_id")
    travel_x_m = LaunchConfiguration("travel_x_m")
    travel_y_m = LaunchConfiguration("travel_y_m")
    travel_yaw_rad = LaunchConfiguration("travel_yaw_rad")

    moon_yard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_simulation", "launch", "moon_yard.launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file("lunabot_bringup", "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "enforce_preflight": "true",
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

    deposit_action_server = Node(
        package="lunabot_control",
        executable="material_action_server",
        name="material_action_server",
        output="screen",
    )

    mission_dry_run = Node(
        package="lunabot_bringup",
        executable="mission_dry_run",
        name="mission_dry_run",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "runtime_preflight_enabled": ParameterValue(
                    runtime_preflight_enabled,
                    value_type=bool,
                ),
                "preflight_config": preflight_config,
                "travel_frame_id": travel_frame_id,
                "travel_x_m": ParameterValue(travel_x_m, value_type=float),
                "travel_y_m": ParameterValue(travel_y_m, value_type=float),
                "travel_yaw_rad": ParameterValue(travel_yaw_rad, value_type=float),
            }
        ],
    )

    harness_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_dry_run,
            on_exit=_handle_harness_exit,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "runtime_preflight_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "preflight_config",
                default_value=_launch_file(
                    "lunabot_bringup",
                    "config",
                    "preflight_checks_dry_run.yaml",
                ),
            ),
            DeclareLaunchArgument("force_overcurrent", default_value="false"),
            DeclareLaunchArgument("force_driver_fault", default_value="false"),
            DeclareLaunchArgument("hold_home_switch_false", default_value="false"),
            DeclareLaunchArgument("travel_frame_id", default_value="map"),
            DeclareLaunchArgument("travel_x_m", default_value="0.5"),
            DeclareLaunchArgument("travel_y_m", default_value="0.0"),
            DeclareLaunchArgument("travel_yaw_rad", default_value="0.0"),
            moon_yard,
            navigation,
            excavation_sim,
            deposit_action_server,
            mission_dry_run,
            harness_exit_handler,
        ]
    )
