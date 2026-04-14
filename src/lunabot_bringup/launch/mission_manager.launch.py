"""Launch the mission manager with arena waypoint configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bringup_config_path(*parts: str) -> str:
    """Return a path inside the lunabot_bringup config share directory."""
    return str(Path(get_package_share_directory("lunabot_bringup")).joinpath("config", *parts))


def generate_launch_description():
    """Generate the mission manager launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoints_file = LaunchConfiguration("waypoints_file")

    mission_manager = Node(
        package="lunabot_bringup",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            waypoints_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock instead of wall time for the mission manager.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=_bringup_config_path("arena_waypoints.yaml"),
                description="Path to arena waypoints YAML for the mission manager.",
            ),
            mission_manager,
        ]
    )
